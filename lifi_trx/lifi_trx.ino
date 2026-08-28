/*
 * ============================================================================
 * ПРОЕКТ: Li-Fi Полнодуплексная оптическая связь (Visible Light Communication)
 * МОДУЛЬ: ТРАНСИВЕР (FULL-DUPLEX + ЦВЕТНОЙ ANSI ИНТЕРФЕЙС + PING + СЖАТИЕ + ARQ)
 * ПЛАТФОРМА: Arduino Uno (ATmega328P)
 * 
 * АППАРАТНАЯ КОНФИГУРАЦИЯ:
 *   - TX (Передатчик):  LED на Pin 13 (через резистор 150-220 Ом на GND)
 *   - RX (Приемник):    Фотодиод BPW24 на Pin A0 (Катод -> 5V, Анод -> A0, Резистор -> GND)
 *   - Связь с ПК:       Hardware Serial UART (115200 baud)
 * 
 * ЦВЕТОВАЯ ПАЛИТРА ANSI:
 *   - 🟢 Зеленый:       Входящие принятые сообщения (RX), Успех, CRC OK
 *   - 🔵 Синий:         Исходящие сообщения (TX)
 *   - 🟣 Пурпурный:     Подтверждения доставки (ACK), Сжатие данных
 *   - 🟡 Желтый:        Оптический Ping, метрики SNR и качества канала
 *   - 🔴 Красный:       Ошибки CRC, перекрытие луча, таймауты
 *   - 🌐 Бирюзовый:     Заголовки и системный статус
 * ============================================================================
 */

#include <Arduino.h>

// ============================================================================
// ANSI ЦВЕТОВЫЕ МАКРОСЫ
// ============================================================================
#define CLR_RESET   "\033[0m"
#define CLR_BOLD    "\033[1m"
#define CLR_RED     "\033[1;31m"
#define CLR_GREEN   "\033[1;32m"
#define CLR_YELLOW  "\033[1;33m"
#define CLR_BLUE    "\033[1;34m"
#define CLR_MAGENTA "\033[1;35m"
#define CLR_CYAN    "\033[1;36m"
#define CLR_WHITE   "\033[1;37m"

// ============================================================================
// КОНФИГУРАЦИЯ И КОНСТАНТЫ
// ============================================================================
namespace Config {
    constexpr uint8_t PIN_TX = 13;                         // Оптический передатчик (LED)
    constexpr uint8_t PIN_RX = A0;                         // Оптический приемник (Фотодиод)

    // Скорость: 45 бод (22.2 мс на бит)
    constexpr uint16_t BAUD_RATE = 45;
    constexpr uint32_t BIT_PERIOD_US = 1000000UL / BAUD_RATE; // 22 222 мкс
    constexpr uint32_t GUARD_PERIOD_US = 3000;             // Защитная пауза: 3 мс

    constexpr size_t TX_QUEUE_SIZE = 256;                  // Размер очереди TX
    constexpr size_t RX_BUFFER_SIZE = 256;                 // Размер буфера RX
    constexpr uint32_t MESSAGE_TIMEOUT_MS = 300;           // Таймаут тишины: 300 мс

    constexpr int TRIGGER_MARGIN = 10;                     // Порог старт-триггера
    constexpr int MIN_SIGNAL_DELTA = 12;                   // Минимальная амплитуда луча
    constexpr int32_t VOTING_OFFSETS_US[3] = {-1500, 0, 1500}; // 3X Оверсэмплинг

    // Управляющие байты протокола ARQ и PING
    constexpr uint8_t CTRL_ACK       = 0x06;               // Байт подтверждения ACK
    constexpr uint8_t CTRL_NAK       = 0x15;               // Байт ошибки NAK
    constexpr uint8_t CTRL_PING_REQ  = 0x05;               // Оптический Ping запрос (ENQ)
    constexpr uint8_t CTRL_PING_RESP = 0x04;               // Оптический Ping ответ (EOT)
    constexpr uint8_t FLAG_COMPRESSED = 0x01;              // Маркер сжатого пакета

    constexpr uint32_t ACK_TIMEOUT_MS = 2500;              // Таймаут ожидания ACK
    constexpr uint8_t MAX_RETRIES = 3;                     // Число повторов ARQ
}

// ============================================================================
// ОЧЕРЕДЬ ПЕРЕДАТЧИКА (TX RING BUFFER)
// ============================================================================
class TxRingBuffer {
private:
    uint8_t buffer[Config::TX_QUEUE_SIZE];
    size_t head = 0;
    size_t tail = 0;
    size_t count = 0;

public:
    bool push(uint8_t b) {
        if (count >= Config::TX_QUEUE_SIZE) return false;
        buffer[tail] = b;
        tail = (tail + 1) % Config::TX_QUEUE_SIZE;
        count++;
        return true;
    }

    bool pop(uint8_t& outByte) {
        if (count == 0) return false;
        outByte = buffer[head];
        head = (head + 1) % Config::TX_QUEUE_SIZE;
        count--;
        return true;
    }

    bool isEmpty() const { return count == 0; }
    size_t size() const { return count; }
    void clear() { head = tail = count = 0; }
};

TxRingBuffer txQueue;

// ============================================================================
// СОСТОЯНИЯ АВТОМАТОВ
// ============================================================================
enum class TxState : uint8_t {
    IDLE,
    START_BIT,
    DATA_BITS,
    STOP_BIT,
    GUARD_PAUSE
};

enum class RxState : uint8_t {
    IDLE_WAIT_FRONT,
    VERIFY_START_BIT,
    SAMPLE_DATA_BITS,
    VERIFY_STOP_BIT,
    COMPLETE_FRAME
};

enum class RxPacketState : uint8_t {
    PAYLOAD,
    WAIT_CRC
};

// ============================================================================
// ПЕРЕМЕННЫЕ ПЕРЕДАТЧИКА, ARQ И PING
// ============================================================================
TxState txState = TxState::IDLE;
uint32_t txBitDeadlineUs = 0;
uint8_t txCurrentByte = 0;
uint8_t txBitIndex = 0;

// Буфер повторной отправки (ARQ)
char lastSentMessage[Config::TX_QUEUE_SIZE];
size_t lastSentMessageLen = 0;
bool isWaitingForAck = false;
uint32_t ackWaitStartTimeMs = 0;
uint8_t currentRetryCount = 0;

// Переменные PING
bool isWaitingForPingReply = false;
uint32_t pingStartTimeMs = 0;

// ============================================================================
// ПЕРЕМЕННЫЕ ПРИЕМНИКА (RX ENGINE)
// ============================================================================
RxState rxState = RxState::IDLE_WAIT_FRONT;
RxPacketState rxPacketState = RxPacketState::PAYLOAD;

uint32_t rxFrameStartUs = 0;
uint8_t rxBitIndex = 0;
uint8_t rxReconstructedByte = 0;
uint8_t rxVotingIndex = 0;
uint8_t rxHighVotesCount = 0;

int ambientNoiseLevel = 3;                                 // Фоновая темнота
int dynamicThreshold = 35;                                // Адаптивный порог
int peakLightAdc = 150;                                   // Пик света старт-бита
int hysteresisVal = 6;

uint8_t rxRawBuffer[Config::RX_BUFFER_SIZE];
size_t rxRawIndex = 0;
uint32_t rxLastCharTimeMs = 0;
uint32_t rxMessageStartTimeMs = 0;
bool rxIsReceivingMessage = false;

long totalLightAdcSum = 0;
int lightSamplesCount = 0;

// ============================================================================
// ПРОТОТИПЫ
// ============================================================================
uint8_t calculateCRC8(const uint8_t* data, size_t len);
size_t compressPayload(const char* src, size_t srcLen, uint8_t* dst, size_t maxDstLen);
size_t decompressPayload(const uint8_t* src, size_t srcLen, char* dst, size_t maxDstLen);

void updateTxEngine();
void updateRxEngine();
void handleSerialInput();
void handleArqTimeouts();
void sendRawPacket(const uint8_t* payload, size_t len, uint8_t crc);
void sendAckFrame(uint8_t controlByte);
void sendPingRequest();
void sendPingResponse();
void calibrateDarkness();
void processReceivedByte(uint8_t byteVal);
void finalizeReceivedMessage(bool hasCRC, uint8_t receivedCRC);

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    pinMode(Config::PIN_TX, OUTPUT);
    digitalWrite(Config::PIN_TX, LOW);

    pinMode(Config::PIN_RX, INPUT);

    Serial.begin(115200);
    delay(400);

    Serial.println(F("\n" CLR_CYAN "============================================================" CLR_RESET));
    Serial.println(F(CLR_BOLD CLR_CYAN " >>> Li-Fi COLOR TRANSCEIVER [FULL-DUPLEX + PING + ARQ] <<< " CLR_RESET));
    Serial.println(F(CLR_CYAN "============================================================" CLR_RESET));
    Serial.print(F(CLR_WHITE "[INFO] Скорость Li-Fi:        " CLR_YELLOW));
    Serial.print(Config::BAUD_RATE);
    Serial.print(F(" бод " CLR_WHITE "| Период бита: " CLR_YELLOW));
    Serial.print(Config::BIT_PERIOD_US / 1000);
    Serial.println(F(" мс" CLR_RESET));
    Serial.println(F(CLR_WHITE "[INFO] Сжатие текста:          " CLR_MAGENTA "ВКЛЮЧЕНО (Кириллица x2)" CLR_RESET));
    Serial.println(F(CLR_WHITE "[INFO] Оптический Ping:        " CLR_YELLOW "ВКЛЮЧЕНО (Команда 'ping')" CLR_RESET));
    Serial.println(F(CLR_WHITE "[INFO] Режим связи:            " CLR_GREEN "FULL-DUPLEX (Одновременный TX/RX)" CLR_RESET));

    calibrateDarkness();

    Serial.println(F(CLR_CYAN "------------------------------------------------------------" CLR_RESET));
    Serial.println(F(CLR_BOLD "Команды:" CLR_RESET));
    Serial.println(F("  - Введите текст для отправки"));
    Serial.println(F("  - " CLR_YELLOW "'ping'" CLR_RESET " - измерить задержку луча (RTT)"));
    Serial.println(F("  - " CLR_CYAN "'c'" CLR_RESET " - калибровка фона, " CLR_CYAN "'r'" CLR_RESET " - замер АЦП"));
    Serial.println(F(CLR_CYAN "------------------------------------------------------------\n" CLR_RESET));
}

// ============================================================================
// ГЛАВНЫЙ ЦИКЛ (LOOP)
// ============================================================================
void loop() {
    handleSerialInput();
    updateTxEngine();
    updateRxEngine();

    if (rxIsReceivingMessage && (millis() - rxLastCharTimeMs > Config::MESSAGE_TIMEOUT_MS)) {
        finalizeReceivedMessage(false, 0);
    }

    handleArqTimeouts();
}

// ============================================================================
// АЛГОРИТМ СЖАТИЯ КИРИЛЛИЦЫ UTF-8
// ============================================================================
size_t compressPayload(const char* src, size_t srcLen, uint8_t* dst, size_t maxDstLen) {
    if (maxDstLen < 2) return 0;
    
    dst[0] = Config::FLAG_COMPRESSED;
    size_t dIdx = 1;

    for (size_t i = 0; i < srcLen && dIdx < maxDstLen - 1; i++) {
        uint8_t c1 = static_cast<uint8_t>(src[i]);

        if (c1 == 0xD0 && i + 1 < srcLen) {
            uint8_t c2 = static_cast<uint8_t>(src[i + 1]);
            if (c2 >= 0x80 && c2 <= 0xBF) {
                dst[dIdx++] = c2;
                i++;
                continue;
            }
        } else if (c1 == 0xD1 && i + 1 < srcLen) {
            uint8_t c2 = static_cast<uint8_t>(src[i + 1]);
            if (c2 >= 0x80 && c2 <= 0xBF) {
                dst[dIdx++] = (c2 + 0x40);
                i++;
                continue;
            }
        }

        dst[dIdx++] = c1;
    }

    return dIdx;
}

size_t decompressPayload(const uint8_t* src, size_t srcLen, char* dst, size_t maxDstLen) {
    if (srcLen == 0 || maxDstLen == 0) return 0;

    size_t dIdx = 0;
    size_t sIdx = (src[0] == Config::FLAG_COMPRESSED) ? 1 : 0;

    while (sIdx < srcLen && dIdx < maxDstLen - 2) {
        uint8_t b = src[sIdx++];

        if (b >= 0x80 && b <= 0xBF) {
            dst[dIdx++] = static_cast<char>(0xD0);
            dst[dIdx++] = static_cast<char>(b);
        } else if (b >= 0xC0 && b <= 0xFF) {
            dst[dIdx++] = static_cast<char>(0xD1);
            dst[dIdx++] = static_cast<char>(b - 0x40);
        } else {
            dst[dIdx++] = static_cast<char>(b);
        }
    }

    dst[dIdx] = '\0';
    return dIdx;
}

// ============================================================================
// РАСЧЕТ CRC-8
// ============================================================================
uint8_t calculateCRC8(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// ============================================================================
// ОТПРАВКА, PING И ARQ
// ============================================================================

void sendRawPacket(const uint8_t* payload, size_t len, uint8_t crc) {
    for (size_t i = 0; i < len; i++) {
        txQueue.push(payload[i]);
    }
    txQueue.push('\n');
    txQueue.push(crc);
}

void sendAckFrame(uint8_t controlByte) {
    txQueue.push(controlByte);
}

void sendPingRequest() {
    isWaitingForPingReply = true;
    pingStartTimeMs = millis();

    Serial.println(F("\n" CLR_YELLOW "========================================================" CLR_RESET));
    Serial.println(F(CLR_BOLD CLR_YELLOW ">>> [OPTICAL PING]: Отправка эхо-запроса по лучу Li-Fi..." CLR_RESET));
    Serial.println(F(CLR_YELLOW "========================================================" CLR_RESET));

    txQueue.push(Config::CTRL_PING_REQ);
}

void sendPingResponse() {
    txQueue.push(Config::CTRL_PING_RESP);
}

void handleSerialInput() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.length() > 0) {
            if (input.equalsIgnoreCase("ping")) {
                sendPingRequest();
                return;
            }

            if (input.length() == 1) {
                char cmd = input.charAt(0);
                if (cmd == 'c' || cmd == 'C') {
                    calibrateDarkness();
                    return;
                } else if (cmd == 'r' || cmd == 'R') {
                    int cur = analogRead(Config::PIN_RX);
                    Serial.print(F(CLR_CYAN "[АЦП]: " CLR_WHITE));
                    Serial.print(cur);
                    Serial.print(F(CLR_CYAN " | Фон: " CLR_WHITE));
                    Serial.print(ambientNoiseLevel);
                    Serial.print(F(CLR_CYAN " | Порог: " CLR_YELLOW));
                    Serial.println(dynamicThreshold);
                    Serial.print(F(CLR_RESET));
                    return;
                }
            }

            size_t copyLen = min(input.length(), sizeof(lastSentMessage) - 1);
            memcpy(lastSentMessage, input.c_str(), copyLen);
            lastSentMessage[copyLen] = '\0';
            lastSentMessageLen = copyLen;

            uint8_t compBuf[Config::TX_QUEUE_SIZE];
            size_t compLen = compressPayload(lastSentMessage, lastSentMessageLen, compBuf, sizeof(compBuf));
            uint8_t crc = calculateCRC8(compBuf, compLen);

            isWaitingForAck = true;
            ackWaitStartTimeMs = millis();
            currentRetryCount = 0;

            Serial.println(F("\n" CLR_BLUE ">>>>>>>>>>>>>> [ TX: ОТПРАВКА СООБЩЕНИЯ ] >>>>>>>>>>>>>>" CLR_RESET));
            Serial.print(F(CLR_WHITE "[TX Текст]:     " CLR_BOLD CLR_BLUE "\""));
            Serial.print(lastSentMessage);
            Serial.println(F("\"" CLR_RESET));
            Serial.print(F(CLR_WHITE "[TX Сжатие]:    Исходный: " CLR_YELLOW));
            Serial.print(lastSentMessageLen);
            Serial.print(F(CLR_WHITE " байт -> В луче: " CLR_MAGENTA));
            Serial.print(compLen);
            Serial.print(F(CLR_WHITE " байт (Экономия: " CLR_GREEN));
            int saved = 100 - (int)((compLen * 100) / lastSentMessageLen);
            Serial.print(max(0, saved));
            Serial.println(F("%) 🗜️" CLR_RESET));
            Serial.print(F(CLR_WHITE "[TX CRC-8]:     " CLR_YELLOW "0x"));
            if (crc < 16) Serial.print(F("0"));
            Serial.println(crc, HEX);
            Serial.println(F(CLR_CYAN "[TX Статус]:    Ожидание подтверждения доставки (ACK)..." CLR_RESET));
            Serial.println(F(CLR_BLUE "--------------------------------------------------------" CLR_RESET));

            sendRawPacket(compBuf, compLen, crc);
        }
    }
}

void handleArqTimeouts() {
    if (isWaitingForPingReply && txQueue.isEmpty() && txState == TxState::IDLE) {
        if (millis() - pingStartTimeMs > 2000) {
            isWaitingForPingReply = false;
            Serial.println(F("\n" CLR_RED "********************************************************" CLR_RESET));
            Serial.println(F(CLR_BOLD CLR_RED ">>> [OPTICAL PING]: Таймаут ответа! Луч не дошел до цели ❌" CLR_RESET));
            Serial.println(F(CLR_RED "********************************************************\n" CLR_RESET));
        }
    }

    if (isWaitingForAck && txQueue.isEmpty() && txState == TxState::IDLE) {
        if (millis() - ackWaitStartTimeMs > Config::ACK_TIMEOUT_MS) {
            if (currentRetryCount < Config::MAX_RETRIES) {
                currentRetryCount++;
                ackWaitStartTimeMs = millis();

                Serial.println(F("\n" CLR_YELLOW "--------------------------------------------------------" CLR_RESET));
                Serial.print(F(CLR_BOLD CLR_YELLOW ">>> [ARQ АВТОПОВТОР]: Таймаут. Повтор пакета (Попытка "));
                Serial.print(currentRetryCount);
                Serial.print(F(" из "));
                Serial.print(Config::MAX_RETRIES);
                Serial.println(F(")... 🔄" CLR_RESET));
                Serial.println(F(CLR_YELLOW "--------------------------------------------------------" CLR_RESET));

                uint8_t compBuf[Config::TX_QUEUE_SIZE];
                size_t compLen = compressPayload(lastSentMessage, lastSentMessageLen, compBuf, sizeof(compBuf));
                uint8_t crc = calculateCRC8(compBuf, compLen);

                sendRawPacket(compBuf, compLen, crc);
            } else {
                isWaitingForAck = false;
                Serial.println(F("\n" CLR_RED "********************************************************" CLR_RESET));
                Serial.println(F(CLR_BOLD CLR_RED ">>> [ДОСТАВКА НЕ УДАЛАСЬ]: Луч перекрыт или нет связи! ❌" CLR_RESET));
                Serial.println(F(CLR_RED "********************************************************\n" CLR_RESET));
            }
        }
    }
}

// ============================================================================
// НЕБЛОКИРУЮЩИЙ АВТОМАТ ПЕРЕДАТЧИКА (TX STATE MACHINE)
// ============================================================================
void updateTxEngine() {
    uint32_t currentUs = micros();

    switch (txState) {
        case TxState::IDLE:
            if (!txQueue.isEmpty()) {
                if (txQueue.pop(txCurrentByte)) {
                    digitalWrite(Config::PIN_TX, HIGH);
                    txBitDeadlineUs = currentUs + Config::BIT_PERIOD_US;
                    txState = TxState::START_BIT;
                }
            }
            break;

        case TxState::START_BIT:
            if ((long)(currentUs - txBitDeadlineUs) >= 0) {
                txBitIndex = 0;
                bool bitVal = (txCurrentByte >> txBitIndex) & 0x01;
                digitalWrite(Config::PIN_TX, bitVal ? HIGH : LOW);
                txBitDeadlineUs += Config::BIT_PERIOD_US;
                txState = TxState::DATA_BITS;
            }
            break;

        case TxState::DATA_BITS:
            if ((long)(currentUs - txBitDeadlineUs) >= 0) {
                txBitIndex++;
                if (txBitIndex < 8) {
                    bool bitVal = (txCurrentByte >> txBitIndex) & 0x01;
                    digitalWrite(Config::PIN_TX, bitVal ? HIGH : LOW);
                    txBitDeadlineUs += Config::BIT_PERIOD_US;
                } else {
                    digitalWrite(Config::PIN_TX, LOW);
                    txBitDeadlineUs += Config::BIT_PERIOD_US;
                    txState = TxState::STOP_BIT;
                }
            }
            break;

        case TxState::STOP_BIT:
            if ((long)(currentUs - txBitDeadlineUs) >= 0) {
                digitalWrite(Config::PIN_TX, LOW);
                txBitDeadlineUs += Config::GUARD_PERIOD_US;
                txState = TxState::GUARD_PAUSE;
            }
            break;

        case TxState::GUARD_PAUSE:
            if ((long)(currentUs - txBitDeadlineUs) >= 0) {
                if (txQueue.isEmpty()) {
                    txState = TxState::IDLE;
                } else {
                    if (txQueue.pop(txCurrentByte)) {
                        digitalWrite(Config::PIN_TX, HIGH);
                        txBitDeadlineUs = currentUs + Config::BIT_PERIOD_US;
                        txState = TxState::START_BIT;
                    } else {
                        txState = TxState::IDLE;
                    }
                }
            }
            break;
    }
}

// ============================================================================
// НЕБЛОКИРУЮЩИЙ АВТОМАТ ПРИЕМНИКА (RX STATE MACHINE)
// ============================================================================
void updateRxEngine() {
    uint32_t currentUs = micros();

    switch (rxState) {
        case RxState::IDLE_WAIT_FRONT: {
            int val = analogRead(Config::PIN_RX);
            if (val > (ambientNoiseLevel + Config::TRIGGER_MARGIN)) {
                rxFrameStartUs = currentUs;
                rxState = RxState::VERIFY_START_BIT;
            }
            break;
        }

        case RxState::VERIFY_START_BIT: {
            uint32_t targetUs = rxFrameStartUs + (Config::BIT_PERIOD_US / 2);
            if ((long)(currentUs - targetUs) >= 0) {
                int sample = analogRead(Config::PIN_RX);
                if (sample <= (ambientNoiseLevel + Config::MIN_SIGNAL_DELTA)) {
                    rxState = RxState::IDLE_WAIT_FRONT;
                } else {
                    peakLightAdc = sample;
                    dynamicThreshold = (ambientNoiseLevel + peakLightAdc) / 2;
                    hysteresisVal = max(4, (peakLightAdc - ambientNoiseLevel) / 10);

                    totalLightAdcSum += peakLightAdc;
                    lightSamplesCount++;

                    rxBitIndex = 0;
                    rxReconstructedByte = 0;
                    rxVotingIndex = 0;
                    rxHighVotesCount = 0;
                    rxState = RxState::SAMPLE_DATA_BITS;
                }
            }
            break;
        }

        case RxState::SAMPLE_DATA_BITS: {
            uint32_t bitCenterUs = rxFrameStartUs + (Config::BIT_PERIOD_US * 3 / 2) + ((uint32_t)rxBitIndex * Config::BIT_PERIOD_US);
            uint32_t sampleTargetUs = bitCenterUs + Config::VOTING_OFFSETS_US[rxVotingIndex];

            if ((long)(currentUs - sampleTargetUs) >= 0) {
                int val = analogRead(Config::PIN_RX);
                if (val >= dynamicThreshold) {
                    rxHighVotesCount++;
                }

                rxVotingIndex++;
                if (rxVotingIndex >= 3) {
                    if (rxHighVotesCount >= 2) {
                        rxReconstructedByte |= (1 << rxBitIndex);
                    }

                    rxBitIndex++;
                    rxVotingIndex = 0;
                    rxHighVotesCount = 0;

                    if (rxBitIndex >= 8) {
                        rxState = RxState::VERIFY_STOP_BIT;
                    }
                }
            }
            break;
        }

        case RxState::VERIFY_STOP_BIT: {
            uint32_t stopTargetUs = rxFrameStartUs + (Config::BIT_PERIOD_US * 19 / 2);
            if ((long)(currentUs - stopTargetUs) >= 0) {
                int stopAdc = analogRead(Config::PIN_RX);
                bool isStopValid = (stopAdc < dynamicThreshold);

                if (isStopValid) {
                    rxState = RxState::COMPLETE_FRAME;
                } else {
                    rxState = RxState::IDLE_WAIT_FRONT;
                }
            }
            break;
        }

        case RxState::COMPLETE_FRAME: {
            uint32_t frameEndUs = rxFrameStartUs + (Config::BIT_PERIOD_US * 10);
            if ((long)(currentUs - frameEndUs) >= 0) {
                processReceivedByte(rxReconstructedByte);
                rxState = RxState::IDLE_WAIT_FRONT;
            }
            break;
        }
    }
}

// ============================================================================
// ОБРАБОТКА ПРИНЯТОГО БАЙТА
// ============================================================================
void processReceivedByte(uint8_t byteVal) {
    if (byteVal == Config::CTRL_PING_REQ) {
        sendPingResponse();
        return;
    }

    if (byteVal == Config::CTRL_PING_RESP) {
        if (isWaitingForPingReply) {
            isWaitingForPingReply = false;
            uint32_t rttMs = millis() - pingStartTimeMs;
            Serial.println();
            Serial.println(F("\n" CLR_GREEN "========================================================" CLR_RESET));
            Serial.println(F(CLR_BOLD CLR_GREEN ">>> [OPTICAL PING]: ОТВЕТ ПОЛУЧЕН! ✔" CLR_RESET));
            Serial.print(F(CLR_WHITE "    • RTT (Круговая задержка): " CLR_BOLD CLR_YELLOW));
            Serial.print(rttMs);
            Serial.println(F(" мс" CLR_RESET));
            Serial.println(F(CLR_GREEN "    • Статус оптического луча: СВЯЗЬ АКТИВНА И СТАБИЛЬНА" CLR_RESET));
            Serial.println(F(CLR_GREEN "========================================================\n" CLR_RESET));
        }
        return;
    }

    if (byteVal == Config::CTRL_ACK) {
        if (isWaitingForAck) {
            isWaitingForAck = false;
            uint32_t rttMs = millis() - ackWaitStartTimeMs;
            Serial.println();
            Serial.println(F("\n" CLR_MAGENTA "========================================================" CLR_RESET));
            Serial.print(F(CLR_BOLD CLR_GREEN ">>> [СТАТУС ДОСТАВКИ]: ПАКЕТ ДОСТАВЛЕН ПОЛУЧАТЕЛЮ! ✔\n" CLR_RESET));
            Serial.print(F(CLR_WHITE ">>> [АРВ-ПОДТВЕРЖДЕНИЕ]: Получен ACK за " CLR_YELLOW));
            Serial.print(rttMs / 1000.0f, 2);
            Serial.println(F(" сек (Целостность 100%)" CLR_RESET));
            Serial.println(F(CLR_MAGENTA "========================================================\n" CLR_RESET));
        }
        return;
    }

    if (byteVal == Config::CTRL_NAK) {
        if (isWaitingForAck) {
            Serial.println(F("\n" CLR_RED ">>> [ARQ]: Получен NAK! Мгновенный повтор..." CLR_RESET));
            ackWaitStartTimeMs = 0;
        }
        return;
    }

    if (!rxIsReceivingMessage) {
        rxIsReceivingMessage = true;
        rxMessageStartTimeMs = millis();
        totalLightAdcSum = 0;
        lightSamplesCount = 0;
    }
    rxLastCharTimeMs = millis();

    if (rxPacketState == RxPacketState::WAIT_CRC) {
        finalizeReceivedMessage(true, byteVal);
        return;
    }

    if (byteVal == '\n' || byteVal == '\r') {
        rxPacketState = RxPacketState::WAIT_CRC;
        return;
    }

    if (rxRawIndex < Config::RX_BUFFER_SIZE - 1) {
        rxRawBuffer[rxRawIndex++] = byteVal;
    }
}

// ============================================================================
// ИТОГОВАЯ ОБРАБОТКА ВХОДЯЩЕГО СООБЩЕНИЯ
// ============================================================================
void finalizeReceivedMessage(bool hasCRC, uint8_t receivedCRC) {
    rxIsReceivingMessage = false;
    rxPacketState = RxPacketState::PAYLOAD;

    if (rxRawIndex == 0) return;

    uint8_t calculatedCRC = calculateCRC8(rxRawBuffer, rxRawIndex);

    char decompressedText[Config::RX_BUFFER_SIZE * 2];
    size_t decompressedLen = decompressPayload(rxRawBuffer, rxRawIndex, decompressedText, sizeof(decompressedText));

    uint32_t totalDurationMs = rxLastCharTimeMs - rxMessageStartTimeMs + 250;
    if (totalDurationMs < 50) totalDurationMs = 50;
    float durationSec = totalDurationMs / 1000.0f;

    float bytesPerSec = static_cast<float>(decompressedLen) / durationSec;
    float bitsPerSec = bytesPerSec * 8.0f;

    int avgLightAdc = (lightSamplesCount > 0) ? static_cast<int>(totalLightAdcSum / lightSamplesCount) : peakLightAdc;
    int contrastDelta = avgLightAdc - ambientNoiseLevel;

    float snrDb = 0.0f;
    int noiseBase = max(1, ambientNoiseLevel);
    if (avgLightAdc > noiseBase) {
        snrDb = 20.0f * log10(static_cast<float>(avgLightAdc) / static_cast<float>(noiseBase));
    }

    Serial.println();
    Serial.println(F("\n" CLR_GREEN "************************************************************" CLR_RESET));
    Serial.print(F(CLR_WHITE ">>> [RX: ПРИНЯТОЕ СООБЩЕНИЕ]: " CLR_BOLD CLR_GREEN "\""));
    Serial.print(decompressedText);
    Serial.println(F("\"" CLR_RESET));
    Serial.print(F(CLR_WHITE ">>> [РАЗМЕР СООБЩЕНИЯ]:       " CLR_YELLOW));
    Serial.print(decompressedLen);
    Serial.print(F(CLR_WHITE " байт (В луче передано: " CLR_MAGENTA));
    Serial.print(rxRawIndex);
    Serial.println(F(" байт) 🗜️" CLR_RESET));

    bool isCrcValid = false;

    if (hasCRC) {
        Serial.print(F(CLR_WHITE ">>> [КОНТРОЛЬ CRC-8]:          Расчетный = " CLR_YELLOW "0x"));
        if (calculatedCRC < 16) Serial.print(F("0"));
        Serial.print(calculatedCRC, HEX);
        Serial.print(F(CLR_WHITE " | Принятый = " CLR_YELLOW "0x"));
        if (receivedCRC < 16) Serial.print(F("0"));
        Serial.println(receivedCRC, HEX);

        if (calculatedCRC == receivedCRC) {
            Serial.println(F(CLR_BOLD CLR_GREEN ">>> [СТАТУС ЦЕЛОСТНОСТИ]:      [УСПЕШНО - ОШИБОК НЕТ!] ✔" CLR_RESET));
            isCrcValid = true;
        } else {
            Serial.println(F(CLR_BOLD CLR_RED ">>> [СТАТУС ЦЕЛОСТНОСТИ]:      [ОШИБКА CRC! ДАННЫЕ ИСКАЖЕНЫ] ❌" CLR_RESET));
        }
    } else {
        Serial.print(F(CLR_RED ">>> [КОНТРОЛЬ CRC-8]:          Таймаут CRC (Расчет: 0x"));
        if (calculatedCRC < 16) Serial.print(F("0"));
        Serial.print(calculatedCRC, HEX);
        Serial.println(F(")" CLR_RESET));
    }

    Serial.println(F(CLR_CYAN "------------------------------------------------------------" CLR_RESET));
    Serial.println(F(CLR_BOLD CLR_CYAN ">>> [МЕТРИКИ ОПТИЧЕСКОГО КАНАЛА LI-FI]:" CLR_RESET));
    Serial.print(F(CLR_WHITE "    • Оптический контраст (ΔV): " CLR_YELLOW));
    Serial.print(contrastDelta);
    Serial.print(F(CLR_WHITE " ADC (Луч: " CLR_YELLOW));
    Serial.print(avgLightAdc);
    Serial.print(F(CLR_WHITE " | Фон: " CLR_WHITE));
    Serial.print(ambientNoiseLevel);
    Serial.println(F(")" CLR_RESET));

    Serial.print(F(CLR_WHITE "    • SNR (Сигнал/Шум):         " CLR_YELLOW));
    Serial.print(snrDb, 1);
    Serial.print(F(" dB "));
    if (snrDb >= 25.0f) {
        Serial.println(F(CLR_GREEN "[ОТЛИЧНЫЙ СИГНАЛ] ★★★" CLR_RESET));
    } else if (snrDb >= 16.0f) {
        Serial.println(F(CLR_YELLOW "[ХОРОШИЙ СИГНАЛ] ★★☆" CLR_RESET));
    } else {
        Serial.println(F(CLR_RED "[СЛАБЫЙ СИГНАЛ] ☆☆☆" CLR_RESET));
    }

    Serial.print(F(CLR_WHITE "    • Эффективная скорость:     " CLR_YELLOW));
    Serial.print(bytesPerSec, 1);
    Serial.print(F(" байт/с (" CLR_YELLOW));
    Serial.print(bitsPerSec, 1);
    Serial.println(F(" бит/с)" CLR_RESET));
    Serial.println(F(CLR_GREEN "************************************************************\n" CLR_RESET));

    if (isCrcValid) {
        sendAckFrame(Config::CTRL_ACK);
    } else {
        sendAckFrame(Config::CTRL_NAK);
    }

    rxRawIndex = 0;
}

// ============================================================================
// КАЛИБРОВКА ТЕМНОТЫ
// ============================================================================
void calibrateDarkness() {
    long sum = 0;
    constexpr int SAMPLES = 60;
    int maxVal = 0;

    for (int i = 0; i < SAMPLES; i++) {
        int val = analogRead(Config::PIN_RX);
        sum += val;
        if (val > maxVal) maxVal = val;
        delay(10);
    }

    ambientNoiseLevel = sum / SAMPLES;
    dynamicThreshold = ambientNoiseLevel + 25;

    Serial.print(F(CLR_CYAN "[КАЛИБРОВКА] Фоновая темнота: " CLR_WHITE));
    Serial.print(ambientNoiseLevel);
    Serial.println(F(" (ADC 0..1023)\n" CLR_RESET));
}
