/*
 * ============================================================================
 * ПРОЕКТ: Li-Fi Полнодуплексная оптическая связь (Laser / VLC)
 * МОДУЛЬ: ТРАНСИВЕР (FULL-DUPLEX + PING + СЖАТИЕ ТЕКСТА + БЫСТРЫЙ ARQ)
 * ПЛАТФОРМА: Arduino Uno (ATmega328P)
 * 
 * АППАРАТНАЯ КОНФИГУРАЦИЯ:
 *   - TX (Передатчик):  Лазерный модуль (KY-008 650нм 5мВт / лазерный диод) на Pin 13
 *   - RX (Приемник):    Фотодиод BPW24 на Pin A0 (Катод -> 5V, Анод -> A0, Резистор -> GND)
 *   - Связь с ПК:       Hardware Serial UART (115200 baud)
 * 
 * ОСОБЕННОСТИ:
 *   - Лазерный оптический канал с высокой коллимацией и дальностью.
 *   - Режим юстировки лазера (команда 'l' или 'laser') со шкалой уровня сигнала.
 *   - Оптический Ping (команда 'ping') с замером RTT.
 *   - Аппаратное сжатие кириллицы UTF-8 (в 2 раза быстрее).
 *   - Надежный протокол ARQ (автоподтверждение ACK и автоповтор).
 * ============================================================================
 */

#include <Arduino.h>

// ============================================================================
// КОНФИГУРАЦИЯ И КОНСТАНТЫ
// ============================================================================
namespace Config {
    constexpr uint8_t PIN_TX = 13;                         // Оптический передатчик: Лазер (KY-008)
    constexpr uint8_t PIN_RX = A0;                         // Оптический приемник: Фотодиод BPW24

    // Скорость: 30 бод (33.3 мс на бит) для идеальной стабильности и совместимости с lifi_tx/lifi_rx
    constexpr uint16_t BAUD_RATE = 30;
    constexpr uint32_t BIT_PERIOD_US = 1000000UL / BAUD_RATE; // 33 333 мкс
    // Защитная пауза: 5 мс (для лазера — чистый спад импульса)
    constexpr uint32_t GUARD_PERIOD_US = 5000;

    constexpr size_t TX_QUEUE_SIZE = 256;                  // Размер очереди TX
    constexpr size_t RX_BUFFER_SIZE = 256;                 // Размер буфера RX
    constexpr uint32_t MESSAGE_TIMEOUT_MS = 500;           // Таймаут тишины: 500 мс

    constexpr int TRIGGER_MARGIN = 10;                     // Порог старт-триггера лазерного импульса
    constexpr int MIN_SIGNAL_DELTA = 15;                   // Минимальная амплитуда лазерного луча
    // Оверсэмплинг ±1200 мкс для 30 бод
    constexpr int32_t VOTING_OFFSETS_US[3] = {-1200, 0, 1200};

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
    IDLE_WAIT_DARK,        // 1. Ожидание темноты (гарантирует, что предыдущий импульс полностью спал)
    IDLE_WAIT_FRONT,       // 2. Линия темная, ждем фронта старт-бита (0 -> 1)
    VERIFY_START_BIT,      // 3. Проверка старт-бита в центре (0.5 T)
    SAMPLE_DATA_BITS,      // 4. Считывание 8 бит данных
    VERIFY_STOP_BIT,       // 5. Проверка стоп-бита (9.5 T)
    COMPLETE_FRAME         // 6. Завершение кадра (10.0 T)
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
RxState rxState = RxState::IDLE_WAIT_DARK;
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

// Режим юстировки лазера (постоянный луч + вывод уровня сигнала)
bool isLaserAimingMode = false;
uint32_t lastAimingPrintMs = 0;

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
void toggleLaserAimingMode();
void updateLaserAiming();
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

    Serial.println(F("\n============================================================"));
    Serial.println(F("  [Li-Fi / LASER ТРАНСИВЕР: FULL-DUPLEX + PING + ARQ]       "));
    Serial.println(F("============================================================"));
    Serial.print(F("[INFO] Скорость Li-Fi:        "));
    Serial.print(Config::BAUD_RATE);
    Serial.print(F(" бод | Длительность бита: "));
    Serial.print(Config::BIT_PERIOD_US / 1000);
    Serial.println(F(" мс"));
    Serial.println(F("[INFO] Оптический канал:       Лазерный луч (KY-008 / 650нм 5мВт)"));
    Serial.println(F("[INFO] Сжатие данных:          ВКЛЮЧЕНО (Кириллица x2)"));
    Serial.println(F("[INFO] Оптический Ping:        ВКЛЮЧЕНО (команда 'ping')"));
    Serial.println(F("[INFO] Режим связи:            FULL-DUPLEX (Одновременный TX/RX)"));

    calibrateDarkness();

    Serial.println(F("------------------------------------------------------------"));
    Serial.println(F("Команды:"));
    Serial.println(F("  - Введите текст для отправки"));
    Serial.println(F("  - 'l' или 'laser' - режим юстировки/наведения луча"));
    Serial.println(F("  - 'ping' - измерить задержку луча (RTT)"));
    Serial.println(F("  - 'c' - калибровка темноты, 'r' - замер АЦП"));
    Serial.println(F("------------------------------------------------------------\n"));
}

// ============================================================================
// ГЛАВНЫЙ ЦИКЛ (LOOP)
// ============================================================================
void loop() {
    handleSerialInput();

    // Если включен режим юстировки лазера, обновляем шкалу сигнала
    if (isLaserAimingMode) {
        updateLaserAiming();
        return;
    }

    updateTxEngine();
    updateRxEngine();

    if (rxIsReceivingMessage && (millis() - rxLastCharTimeMs > Config::MESSAGE_TIMEOUT_MS)) {
        finalizeReceivedMessage(false, 0);
    }

    handleArqTimeouts();
}

// ============================================================================
// АЛГОРИТМ СЖАТИЯ (КИРИЛЛИЦА UTF-8 PACKING)
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
// РАСЧЕТ CRC-8 (Полином 0x07)
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

    Serial.println(F("\n========================================================"));
    Serial.println(F("[OPTICAL PING]: Отправка эхо-запроса по лучу Li-Fi..."));
    Serial.println(F("========================================================"));

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
            // Режим юстировки лазера (наведение луча)
            if (input.equalsIgnoreCase("laser") || input.equalsIgnoreCase("l")) {
                toggleLaserAimingMode();
                return;
            }

            // Команда PING
            if (input.equalsIgnoreCase("ping")) {
                sendPingRequest();
                return;
            }

            // Служебные команды
            if (input.length() == 1) {
                char cmd = input.charAt(0);
                if (cmd == 'c' || cmd == 'C') {
                    calibrateDarkness();
                    return;
                } else if (cmd == 'r' || cmd == 'R') {
                    int cur = analogRead(Config::PIN_RX);
                    int delta = cur - ambientNoiseLevel;
                    Serial.println(F("\n--- [ДИАГНОСТИКА СИГНАЛА АЦП A0] ---"));
                    Serial.print(F("Текущий уровень АЦП:  ")); Serial.println(cur);
                    Serial.print(F("Фоновая темнота:      ")); Serial.println(ambientNoiseLevel);
                    Serial.print(F("Амплитуда луча (ΔV):  ")); Serial.println(delta);
                    Serial.print(F("Порог срабатывания:   ")); Serial.println(ambientNoiseLevel + Config::TRIGGER_MARGIN);
                    if (delta >= Config::MIN_SIGNAL_DELTA) {
                        Serial.println(F("СТАТУС: [СИГНАЛ В НОРМЕ] Луч уверенно регистрируется!"));
                    } else if (delta > 0) {
                        Serial.println(F("СТАТУС: [СЛАБЫЙ СИГНАЛ] Амплитуды недостаточно для надежного приема."));
                        Serial.println(F("СОВЕТ: Точнее направьте лазер в фотодиод или увеличьте резистор подтяжки (5-20 кОм)."));
                    } else if (delta < -20) {
                        Serial.println(F("СТАТУС: [ИНВЕРТИРОВАННАЯ ПОЛЯРНОСТЬ] При свете АЦП падает, а не растет!"));
                        Serial.println(F("СОВЕТ: Поменяйте выводы фотодиода (Катод -> 5V, Анод -> A0, Резистор -> GND)."));
                    } else {
                        Serial.println(F("СТАТУС: [НЕТ СИГНАЛА] Фотодиод не видит свет лазера."));
                    }
                    Serial.println(F("------------------------------------\n"));
                    return;
                }
            }

            size_t copyLen = min(input.length(), sizeof(lastSentMessage) - 1);
            memcpy(lastSentMessage, input.c_str(), copyLen);
            lastSentMessage[copyLen] = '\0';
            lastSentMessageLen = copyLen;

            // Сжимаем текст перед отправкой
            uint8_t compBuf[Config::TX_QUEUE_SIZE];
            size_t compLen = compressPayload(lastSentMessage, lastSentMessageLen, compBuf, sizeof(compBuf));

            uint8_t crc = calculateCRC8(compBuf, compLen);

            isWaitingForAck = true;
            ackWaitStartTimeMs = millis();
            currentRetryCount = 0;

            Serial.println(F("\n>>>>>>>>>>>>>> [ TX: ОТПРАВКА СООБЩЕНИЯ ] >>>>>>>>>>>>>>"));
            Serial.print(F("[TX Текст]:     \""));
            Serial.print(lastSentMessage);
            Serial.println(F("\""));
            Serial.print(F("[TX Сжатие]:    Исходный: "));
            Serial.print(lastSentMessageLen);
            Serial.print(F(" байт -> В луче: "));
            Serial.print(compLen);
            Serial.print(F(" байт (Экономия: "));
            int saved = 100 - (int)((compLen * 100) / lastSentMessageLen);
            Serial.print(max(0, saved));
            Serial.println(F("%)"));
            Serial.print(F("[TX CRC-8]:     0x"));
            if (crc < 16) Serial.print(F("0"));
            Serial.println(crc, HEX);
            Serial.println(F("[TX Статус]:    Ожидание подтверждения доставки (ACK)..."));
            Serial.println(F("--------------------------------------------------------"));

            sendRawPacket(compBuf, compLen, crc);
        }
    }
}

void handleArqTimeouts() {
    // 1. Проверка таймаута PING
    if (isWaitingForPingReply && txQueue.isEmpty() && txState == TxState::IDLE) {
        if (millis() - pingStartTimeMs > 2000) {
            isWaitingForPingReply = false;
            Serial.println(F("\n********************************************************"));
            Serial.println(F("[OPTICAL PING]: Таймаут ответа! Луч не дошел до цели."));
            Serial.println(F("********************************************************\n"));
        }
    }

    // 2. Проверка таймаута ARQ
    if (isWaitingForAck && txQueue.isEmpty() && txState == TxState::IDLE) {
        if (millis() - ackWaitStartTimeMs > Config::ACK_TIMEOUT_MS) {
            if (currentRetryCount < Config::MAX_RETRIES) {
                currentRetryCount++;
                ackWaitStartTimeMs = millis();

                Serial.println(F("\n--------------------------------------------------------"));
                Serial.print(F("[ARQ АВТОПОВТОР]: Таймаут. Повтор пакета (Попытка "));
                Serial.print(currentRetryCount);
                Serial.print(F(" из "));
                Serial.print(Config::MAX_RETRIES);
                Serial.println(F(")..."));
                Serial.println(F("--------------------------------------------------------"));

                uint8_t compBuf[Config::TX_QUEUE_SIZE];
                size_t compLen = compressPayload(lastSentMessage, lastSentMessageLen, compBuf, sizeof(compBuf));
                uint8_t crc = calculateCRC8(compBuf, compLen);

                sendRawPacket(compBuf, compLen, crc);
            } else {
                isWaitingForAck = false;
                Serial.println(F("\n********************************************************"));
                Serial.println(F("[ДОСТАВКА НЕ УДАЛАСЬ]: Луч перекрыт или нет связи."));
                Serial.println(F("********************************************************\n"));
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

    // Автоматическая адаптация к комнатному свету в моменты покоя
    static uint32_t lastBaselineTrackMs = 0;
    if (!rxIsReceivingMessage && (rxState == RxState::IDLE_WAIT_FRONT || rxState == RxState::IDLE_WAIT_DARK)) {
        if (millis() - lastBaselineTrackMs >= 100) {
            lastBaselineTrackMs = millis();
            int curBg = analogRead(Config::PIN_RX);
            if (curBg < dynamicThreshold) {
                ambientNoiseLevel = (ambientNoiseLevel * 3 + curBg) / 4;
                int expectedPeak = max(peakLightAdc, ambientNoiseLevel + 150);
                dynamicThreshold = ambientNoiseLevel + (int)(((long)(expectedPeak - ambientNoiseLevel) * 65) / 100);
            }
        }
    }

    switch (rxState) {
        case RxState::IDLE_WAIT_DARK: {
            int val = analogRead(Config::PIN_RX);
            // Перед началом любого приема канал ОБЯЗАН быть темным (ниже динамического порога).
            if (val < dynamicThreshold) {
                rxState = RxState::IDLE_WAIT_FRONT;
            } else if ((long)(currentUs - rxFrameStartUs) > (long)(Config::BIT_PERIOD_US * 5)) {
                // Защита от зависания: если свет не падает дольше 5 периодов бита (например, включили люстру),
                // пересчитываем порог и разблокируем прием
                ambientNoiseLevel = val;
                peakLightAdc = max(1000, ambientNoiseLevel + 150);
                dynamicThreshold = ambientNoiseLevel + (int)(((long)(peakLightAdc - ambientNoiseLevel) * 65) / 100);
                rxState = RxState::IDLE_WAIT_FRONT;
            }
            break;
        }

        case RxState::IDLE_WAIT_FRONT: {
            int val = analogRead(Config::PIN_RX);
            // Фронт старт-бита: переход 0 -> 1 (луч включился и превысил порог)
            if (val >= dynamicThreshold) {
                rxFrameStartUs = currentUs;
                rxState = RxState::VERIFY_START_BIT;
            }
            break;
        }

        case RxState::VERIFY_START_BIT: {
            uint32_t targetUs = rxFrameStartUs + (Config::BIT_PERIOD_US / 2);
            if ((long)(currentUs - targetUs) >= 0) {
                int sample = analogRead(Config::PIN_RX);
                // В центре старт-бита уровень ОБЯЗАН быть выше порога!
                if (sample < dynamicThreshold) {
                    rxState = RxState::IDLE_WAIT_DARK;
                } else {
                    peakLightAdc = sample;
                    // Порог ставим на 65% от размаха (ближе к лазеру),
                    // чтобы спадающий луч мгновенно пересекал порог сверху вниз в первые же миллисекунды!
                    dynamicThreshold = ambientNoiseLevel + (int)(((long)(peakLightAdc - ambientNoiseLevel) * 65) / 100);
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
            // Сэмплируем на 62.5% длительности бита (1.625 T вместо 1.5 T),
            // давая фотодиоду дополнительное время на полный разряд при передаче нулей
            uint32_t bitCenterUs = rxFrameStartUs + (Config::BIT_PERIOD_US * 13 / 8) + ((uint32_t)rxBitIndex * Config::BIT_PERIOD_US);
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
                rxState = RxState::COMPLETE_FRAME;
            }
            break;
        }

        case RxState::COMPLETE_FRAME: {
            uint32_t frameEndUs = rxFrameStartUs + (Config::BIT_PERIOD_US * 10);
            if ((long)(currentUs - frameEndUs) >= 0) {
                processReceivedByte(rxReconstructedByte);
                // После приема кадра переходим в ожидание темноты
                rxState = RxState::IDLE_WAIT_DARK;
            }
            break;
        }
    }
}

// ============================================================================
// ОБРАБОТКА ПРИНЯТОГО БАЙТА
// ============================================================================
void processReceivedByte(uint8_t byteVal) {
    // 1. Служебный байт PING REQUEST -> мгновенный ответ
    if (byteVal == Config::CTRL_PING_REQ) {
        sendPingResponse();
        return;
    }

    // 2. Служебный байт PING RESPONSE -> фиксация RTT
    if (byteVal == Config::CTRL_PING_RESP) {
        if (isWaitingForPingReply) {
            isWaitingForPingReply = false;
            uint32_t rttMs = millis() - pingStartTimeMs;
            Serial.println();
            Serial.println(F("\n========================================================"));
            Serial.println(F(">>> [OPTICAL PING]: ОТВЕТ ПОЛУЧЕН [OK]"));
            Serial.print(F("    - RTT (Круговая задержка): "));
            Serial.print(rttMs);
            Serial.println(F(" мс"));
            Serial.println(F("    - Статус: СВЯЗЬ АКТИВНА И СТАБИЛЬНА"));
            Serial.println(F("========================================================\n"));
        }
        return;
    }

    // 3. Служебный байт ACK
    if (byteVal == Config::CTRL_ACK) {
        if (isWaitingForAck) {
            isWaitingForAck = false;
            uint32_t rttMs = millis() - ackWaitStartTimeMs;
            Serial.println();
            Serial.println(F("\n========================================================"));
            Serial.print(F(">>> [СТАТУС ДОСТАВКИ]: ПАКЕТ ДОСТАВЛЕН ПОЛУЧАТЕЛЮ [OK]\n"));
            Serial.print(F(">>> [ARQ-ПОДТВЕРЖДЕНИЕ]: Получен ACK за "));
            Serial.print(rttMs / 1000.0f, 2);
            Serial.println(F(" сек"));
            Serial.println(F("========================================================\n"));
        }
        return;
    }

    // 4. Служебный байт NAK
    if (byteVal == Config::CTRL_NAK) {
        if (isWaitingForAck) {
            Serial.println(F("\n>>> [ARQ]: Получен NAK. Мгновенный повтор..."));
            ackWaitStartTimeMs = 0;
        }
        return;
    }

    if (!rxIsReceivingMessage) {
        rxIsReceivingMessage = true;
        rxMessageStartTimeMs = millis();
        totalLightAdcSum = 0;
        lightSamplesCount = 0;
        Serial.print(F("\n[RX]: Прием данных... "));
    }
    rxLastCharTimeMs = millis();

    // 5. Ожидание контрольного байта CRC-8
    if (rxPacketState == RxPacketState::WAIT_CRC) {
        Serial.print(F(" [CRC: 0x"));
        if (byteVal < 16) Serial.print(F("0"));
        Serial.print(byteVal, HEX);
        Serial.println(F("]"));
        finalizeReceivedMessage(true, byteVal);
        return;
    }

    // 6. Маркер окончания текста
    if (byteVal == '\n' || byteVal == '\r') {
        rxPacketState = RxPacketState::WAIT_CRC;
        return;
    }

    // Выводим принятый байт в реальном времени (HEX и символ)
    Serial.print(F(" 0x"));
    if (byteVal < 16) Serial.print(F("0"));
    Serial.print(byteVal, HEX);
    if (byteVal >= 32 && byteVal <= 126) {
        Serial.print(F("('"));
        Serial.print((char)byteVal);
        Serial.print(F("')"));
    }

    // Сохраняем сырой байт в буфер
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

    // Расчет CRC-8 от принятого сжатого пакета
    uint8_t calculatedCRC = calculateCRC8(rxRawBuffer, rxRawIndex);

    // Распаковываем текст обратно в UTF-8
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
    Serial.println(F("\n************************************************************"));
    Serial.print(F(">>> [RX: ПРИНЯТОЕ СООБЩЕНИЕ]: \""));
    Serial.print(decompressedText);
    Serial.println(F("\""));
    Serial.print(F(">>> [РАЗМЕР СООБЩЕНИЯ]:       "));
    Serial.print(decompressedLen);
    Serial.print(F(" байт (В луче передано: "));
    Serial.print(rxRawIndex);
    Serial.println(F(" байт)"));

    bool isCrcValid = false;

    if (hasCRC) {
        Serial.print(F(">>> [КОНТРОЛЬ CRC-8]:          Расчетный = 0x"));
        if (calculatedCRC < 16) Serial.print(F("0"));
        Serial.print(calculatedCRC, HEX);
        Serial.print(F(" | Принятый = 0x"));
        if (receivedCRC < 16) Serial.print(F("0"));
        Serial.println(receivedCRC, HEX);

        if (calculatedCRC == receivedCRC) {
            Serial.println(F(">>> [СТАТУС ЦЕЛОСТНОСТИ]:      [УСПЕШНО - ОШИБОК НЕТ]"));
            isCrcValid = true;
        } else {
            Serial.println(F(">>> [СТАТУС ЦЕЛОСТНОСТИ]:      [ОШИБКА CRC: ДАННЫЕ ИСКАЖЕНЫ]"));
        }
    } else {
        Serial.print(F(">>> [КОНТРОЛЬ CRC-8]:          Таймаут CRC (Расчет: 0x"));
        if (calculatedCRC < 16) Serial.print(F("0"));
        Serial.print(calculatedCRC, HEX);
        Serial.println(F(")"));
    }

    // ТЕЛЕМЕТРИЯ КАНАЛА
    Serial.println(F("------------------------------------------------------------"));
    Serial.println(F(">>> [МЕТРИКИ ОПТИЧЕСКОГО КАНАЛА LI-FI]:"));
    Serial.print(F("    - Оптический контраст (dV): "));
    Serial.print(contrastDelta);
    Serial.print(F(" ADC (Луч: "));
    Serial.print(avgLightAdc);
    Serial.print(F(" | Фон: "));
    Serial.print(ambientNoiseLevel);
    Serial.println(F(")"));

    Serial.print(F("    - SNR (Сигнал/Шум):         "));
    Serial.print(snrDb, 1);
    Serial.print(F(" dB "));
    if (snrDb >= 25.0f) {
        Serial.println(F("[ОТЛИЧНЫЙ СИГНАЛ]"));
    } else if (snrDb >= 16.0f) {
        Serial.println(F("[ХОРОШИЙ СИГНАЛ]"));
    } else {
        Serial.println(F("[СЛАБЫЙ СИГНАЛ]"));
    }

    Serial.print(F("    - Скорость передачи:        "));
    Serial.print(bytesPerSec, 1);
    Serial.print(F(" байт/с ("));
    Serial.print(bitsPerSec, 1);
    Serial.println(F(" бит/с)"));
    Serial.println(F("************************************************************\n"));

    if (isCrcValid) {
        sendAckFrame(Config::CTRL_ACK);
    } else {
        sendAckFrame(Config::CTRL_NAK);
    }

    rxRawIndex = 0;
}

// ==========================================
// КАЛИБРОВКА ТЕМНОТЫ
// ==========================================
void calibrateDarkness() {
    digitalWrite(Config::PIN_TX, LOW); // Гарантируем отключение своего лазера
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
    peakLightAdc = max(1000, ambientNoiseLevel + 200);
    dynamicThreshold = ambientNoiseLevel + (int)(((long)(peakLightAdc - ambientNoiseLevel) * 65) / 100);

    Serial.print(F("[КАЛИБРОВКА] Фоновый свет комнаты: "));
    Serial.print(ambientNoiseLevel);
    Serial.print(F(" | Порог приема: "));
    Serial.print(dynamicThreshold);
    Serial.println(F(" (ADC 0..1023)\n"));
}

// ==========================================
// РЕЖИМ ЮСТИРОВКИ (НАВЕДЕНИЯ) ЛАЗЕРА
// ==========================================
void toggleLaserAimingMode() {
    isLaserAimingMode = !isLaserAimingMode;
    if (isLaserAimingMode) {
        txQueue.clear();
        txState = TxState::IDLE;
        rxState = RxState::IDLE_WAIT_FRONT;
        digitalWrite(Config::PIN_TX, HIGH); // Включаем лазер непрерывно для юстировки

        Serial.println(F("\n========================================================"));
        Serial.println(F(">>> [РЕЖИМ ЮСТИРОВКИ ЛАЗЕРА ВКЛЮЧЕН] <<<"));
        Serial.println(F("Красный луч лазера включен непрерывно."));
        Serial.println(F("Направьте луч на фотодиод приемника, ориентируясь на шкалу АЦП."));
        Serial.println(F("Для выхода из юстировки введите 'l' или 'laser'."));
        Serial.println(F("========================================================"));
    } else {
        digitalWrite(Config::PIN_TX, LOW); // Выключаем лазер
        Serial.println(F("\n>>> [РЕЖИМ ЮСТИРОВКИ ВЫКЛЮЧЕН]"));
        calibrateDarkness();
        Serial.println(F("Трансивер готов к передаче и приему данных.\n"));
    }
}

void updateLaserAiming() {
    if (millis() - lastAimingPrintMs >= 200) {
        lastAimingPrintMs = millis();
        int cur = analogRead(Config::PIN_RX);

        Serial.print(F("[ЮСТИРОВКА] АЦП: "));
        if (cur < 1000) Serial.print(F(" "));
        if (cur < 100) Serial.print(F(" "));
        if (cur < 10) Serial.print(F(" "));
        Serial.print(cur);
        Serial.print(F(" ["));

        int bars = map(constrain(cur, ambientNoiseLevel, 1023), ambientNoiseLevel, 1023, 0, 20);
        for (int b = 0; b < 20; b++) {
            Serial.print(b < bars ? '=' : ' ');
        }
        Serial.print(F("] "));

        if (cur > (ambientNoiseLevel + Config::MIN_SIGNAL_DELTA)) {
            Serial.println(F(">> ЛУЧ ПОЙМАН <<"));
        } else {
            Serial.println(F("нет луча"));
        }
    }
}

