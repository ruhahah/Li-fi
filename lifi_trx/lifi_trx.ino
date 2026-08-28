/*
 * ============================================================================
 * ПРОЕКТ: Li-Fi Полнодуплексная оптическая связь (Visible Light Communication)
 * МОДУЛЬ: ТРАНСИВЕР (FULL-DUPLEX + ПРОТОКОЛ ARQ / ACK + ЗВУКОВАЯ ТЕЛЕМЕТРИЯ)
 * ПЛАТФОРМА: Arduino Uno (ATmega328P)
 * 
 * АППАРАТНАЯ КОНФИГУРАЦИЯ:
 *   - TX (Передатчик):  LED на Pin 13 (через резистор 150-220 Ом на GND)
 *   - RX (Приемник):    Фотодиод BPW24 на Pin A0 (Катод -> 5V, Анод -> A0, Резистор -> GND)
 *   - BUZZER (Звук):    Пьезодинамик / Зуммер на Pin 8 (Плюс -> Pin 8, Минус -> GND)
 *   - Связь с ПК:       Hardware Serial UART (115200 baud)
 * 
 * НОВЫЕ ВОЗМОЖНОСТИ В ЭТОЙ ВЕРСИИ:
 *   1. Протокол гарантированной доставки ARQ (ACK / NAK / Auto-Retransmit):
 *      - Автоматическое оптическое подтверждение доставки (ACK).
 *      - Автоматический повтор отправки при помехах или перекрытии луча (до 3 попыток).
 *   2. Звуковая индикация событий (Buzzer):
 *      - Приятный аккорд при подтверждении доставки (ACK).
 *      - Звуковой отклик на входящие сообщения и отправку.
 *      - Тревожный сигнал при обнаружении искажений CRC-8.
 *   3. Полнодуплексный неблокирующий движок (Zero CPU Delay).
 * ============================================================================
 */

#include <Arduino.h>

// ============================================================================
// КОНФИГУРАЦИЯ И КОНСТАНТЫ
// ============================================================================
namespace Config {
    constexpr uint8_t PIN_TX = 13;                         // Оптический передатчик (LED)
    constexpr uint8_t PIN_RX = A0;                         // Оптический приемник (Фотодиод)
    constexpr uint8_t PIN_BUZZER = 8;                      // Зуммер / Пьезодинамик (опционально)

    constexpr uint16_t BAUD_RATE = 30;                     // Скорость: 30 бит/с
    constexpr uint32_t BIT_PERIOD_US = 1000000UL / BAUD_RATE; // 33 333 мкс (1 бит)
    constexpr uint32_t GUARD_PERIOD_US = BIT_PERIOD_US;    // 33 333 мкс (пауза между байтами)

    constexpr size_t TX_QUEUE_SIZE = 256;                  // Размер очереди TX
    constexpr size_t RX_BUFFER_SIZE = 256;                 // Размер буфера RX
    constexpr uint32_t MESSAGE_TIMEOUT_MS = 500;           // Таймаут тишины окончания фразы

    constexpr int TRIGGER_MARGIN = 12;                     // Порог старт-триггера над фоном
    constexpr int MIN_SIGNAL_DELTA = 15;                   // Минимальная амплитуда луча
    constexpr int32_t VOTING_OFFSETS_US[3] = {-2000, 0, 2000}; // 3X Оверсэмплинг

    // Управляющие байты протокола ARQ
    constexpr uint8_t CTRL_ACK = 0x06;                     // Байт подтверждения успешной доставки
    constexpr uint8_t CTRL_NAK = 0x15;                     // Байт ошибки контрольной суммы
    constexpr uint32_t ACK_TIMEOUT_MS = 3500;              // Таймаут ожидания подтверждения ACK
    constexpr uint8_t MAX_RETRIES = 3;                     // Максимальное число автоповторов
}

// ============================================================================
// ЗВУКОВОЙ ДИСПЕТЧЕР (SOUND EFFECTS)
// ============================================================================
namespace Sound {
    void playTxSent() {
        tone(Config::PIN_BUZZER, 1200, 40);
    }

    void playRxReceived() {
        tone(Config::PIN_BUZZER, 1500, 50);
    }

    void playAckConfirmed() {
        tone(Config::PIN_BUZZER, 1400, 50);
        delay(60);
        tone(Config::PIN_BUZZER, 2100, 100);
    }

    void playCrcError() {
        tone(Config::PIN_BUZZER, 350, 200);
    }

    void playRetryAlert() {
        tone(Config::PIN_BUZZER, 800, 70);
    }
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
// ПЕРЕМЕННЫЕ ПЕРЕДАТЧИКА И ARQ
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
int hysteresisVal = 8;

char rxSentenceBuffer[Config::RX_BUFFER_SIZE];
size_t rxBufferIndex = 0;
uint32_t rxLastCharTimeMs = 0;
uint32_t rxMessageStartTimeMs = 0;
bool rxIsReceivingMessage = false;

long totalLightAdcSum = 0;
int lightSamplesCount = 0;

// ============================================================================
// ПРОТОТИПЫ
// ============================================================================
uint8_t calculateCRC8(const uint8_t* data, size_t len);
void updateTxEngine();
void updateRxEngine();
void handleSerialInput();
void handleArqTimeouts();
void sendRawPacket(const char* text, size_t len);
void sendAckFrame(uint8_t controlByte);
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
    pinMode(Config::PIN_BUZZER, OUTPUT);

    Serial.begin(115200);
    delay(500);

    Serial.println(F("\n============================================================"));
    Serial.println(F("  >>> Li-Fi ТРАНСИВЕР [FULL-DUPLEX + ПРОТОКОЛ ARQ/ACK] <<<  "));
    Serial.println(F("============================================================"));
    Serial.print(F("[INFO] Скорость Li-Fi:        "));
    Serial.print(Config::BAUD_RATE);
    Serial.print(F(" бод | Длительность бита: "));
    Serial.print(Config::BIT_PERIOD_US / 1000);
    Serial.println(F(" мс"));
    Serial.println(F("[INFO] Режим связи:            FULL-DUPLEX (Одновременный TX/RX)"));
    Serial.println(F("[INFO] Гарантия доставки:      Протокол ARQ (ACK / NAK / Auto-Retransmit)"));
    Serial.println(F("[INFO] Звуковая индикация:     Пьезодинамик на Pin 8"));
    Serial.println(F("[INFO] Поддержка языков:       UTF-8 Русский + English + CRC-8"));

    calibrateDarkness();

    // Приветственный звуковой сигнал готовности
    tone(Config::PIN_BUZZER, 1000, 80);
    delay(100);
    tone(Config::PIN_BUZZER, 1500, 100);

    Serial.println(F("------------------------------------------------------------"));
    Serial.println(F("Трансивер готов к надежной связи с подтверждением доставки!\n"));
}

// ============================================================================
// ГЛАВНЫЙ ЦИКЛ (LOOP)
// ============================================================================
void loop() {
    // 1. Опрос ввода с ПК
    handleSerialInput();

    // 2. Шаг автомата передатчика (TX)
    updateTxEngine();

    // 3. Шаг автомата приемника (RX)
    updateRxEngine();

    // 4. Проверка таймаута тишины завершения входящего сообщения
    if (rxIsReceivingMessage && (millis() - rxLastCharTimeMs > Config::MESSAGE_TIMEOUT_MS)) {
        finalizeReceivedMessage(false, 0);
    }

    // 5. Проверка таймаутов протокола ARQ (ожидание подтверждения ACK / автоповтор)
    handleArqTimeouts();
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
// ПРОТОКОЛ ARQ: ОТПРАВКА И УПРАВЛЕНИЕ ДОСТАВКОЙ
// ============================================================================

void sendRawPacket(const char* text, size_t len) {
    uint8_t crc = calculateCRC8(reinterpret_cast<const uint8_t*>(text), len);

    // 1. Помещаем байты текста
    for (size_t i = 0; i < len; i++) {
        txQueue.push(static_cast<uint8_t>(text[i]));
    }
    // 2. Маркер окончания строки
    txQueue.push('\n');
    // 3. Байт CRC-8
    txQueue.push(crc);

    Sound::playTxSent();
}

void sendAckFrame(uint8_t controlByte) {
    // Служебный пакет подтверждения: байт управления + маркер
    txQueue.push(controlByte);
    txQueue.push('\n');
    txQueue.push(controlByte); // CRC для служебного кадра
}

void handleSerialInput() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.length() > 0) {
            // Быстрые служебные команды
            if (input.length() == 1) {
                char cmd = input.charAt(0);
                if (cmd == 'c' || cmd == 'C') {
                    calibrateDarkness();
                    return;
                } else if (cmd == 'r' || cmd == 'R') {
                    int cur = analogRead(Config::PIN_RX);
                    Serial.print(F("[АЦП]: "));
                    Serial.print(cur);
                    Serial.print(F(" | Фон: "));
                    Serial.print(ambientNoiseLevel);
                    Serial.print(F(" | Порог: "));
                    Serial.println(dynamicThreshold);
                    return;
                }
            }

            // Сохраняем сообщение в буфер ARQ для возможного автоповтора
            size_t copyLen = min(input.length(), sizeof(lastSentMessage) - 1);
            memcpy(lastSentMessage, input.c_str(), copyLen);
            lastSentMessage[copyLen] = '\0';
            lastSentMessageLen = copyLen;

            isWaitingForAck = true;
            ackWaitStartTimeMs = millis();
            currentRetryCount = 0;

            uint8_t crc = calculateCRC8(reinterpret_cast<const uint8_t*>(lastSentMessage), lastSentMessageLen);

            Serial.println(F("\n>>>>>>>>>>>>>> [ TX: ОТПРАВКА СООБЩЕНИЯ ] >>>>>>>>>>>>>>"));
            Serial.print(F("[TX Текст]:    \""));
            Serial.print(lastSentMessage);
            Serial.println(F("\""));
            Serial.print(F("[TX Объем]:    "));
            Serial.print(lastSentMessageLen);
            Serial.print(F(" байт | CRC-8: 0x"));
            if (crc < 16) Serial.print(F("0"));
            Serial.println(crc, HEX);
            Serial.println(F("[TX Статус]:   Ожидание подтверждения доставки (ACK)..."));
            Serial.println(F("--------------------------------------------------------"));

            // Отправляем пакет в оптический канал
            sendRawPacket(lastSentMessage, lastSentMessageLen);
        }
    }
}

void handleArqTimeouts() {
    if (isWaitingForAck && txQueue.isEmpty() && txState == TxState::IDLE) {
        if (millis() - ackWaitStartTimeMs > Config::ACK_TIMEOUT_MS) {
            // Время ожидания ACK истекло
            if (currentRetryCount < Config::MAX_RETRIES) {
                currentRetryCount++;
                ackWaitStartTimeMs = millis();

                Serial.println(F("\n--------------------------------------------------------"));
                Serial.print(F(">>> [ARQ АВТОПОВТОР]: Таймаут ответа. Повторная отправка пакета (Попытка "));
                Serial.print(currentRetryCount);
                Serial.print(F(" из "));
                Serial.print(Config::MAX_RETRIES);
                Serial.println(F(")... 🔄"));
                Serial.println(F("--------------------------------------------------------"));

                Sound::playRetryAlert();
                sendRawPacket(lastSentMessage, lastSentMessageLen);
            } else {
                // Превышен лимит попыток
                isWaitingForAck = false;
                Serial.println(F("\n********************************************************"));
                Serial.println(F(">>> [ДОСТАВКА НЕ УДАЛАСЬ]: Луч перекрыт или нет связи! ❌"));
                Serial.println(F("********************************************************\n"));
                Sound::playCrcError();
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
    // Перехват служебных пакетов подтверждения ARQ (ACK / NAK)
    if (byteVal == Config::CTRL_ACK) {
        if (isWaitingForAck) {
            isWaitingForAck = false;
            uint32_t rttMs = millis() - ackWaitStartTimeMs;
            Serial.println();
            Serial.println(F("\n========================================================"));
            Serial.print(F(">>> [СТАТУС ДОСТАВКИ]: ПАКЕТ ДОСТАВЛЕН ПОЛУЧАТЕЛЮ! ✔\n"));
            Serial.print(F(">>> [АРВ-ПОДТВЕРЖДЕНИЕ]: Получен ACK за "));
            Serial.print(rttMs / 1000.0f, 2);
            Serial.println(F(" сек (Целостность 100%)"));
            Serial.println(F("========================================================\n"));
            Sound::playAckConfirmed();
        }
        return;
    } else if (byteVal == Config::CTRL_NAK) {
        if (isWaitingForAck) {
            Serial.println(F("\n>>> [ARQ]: Получен сигнал ошибки (NAK) от собеседника! Запуск повтора..."));
            // Принудительно запускаем таймаут для мгновенного повтора
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

    // 1. Ожидание контрольного байта CRC-8
    if (rxPacketState == RxPacketState::WAIT_CRC) {
        finalizeReceivedMessage(true, byteVal);
        return;
    }

    // 2. Маркер окончания текста
    if (byteVal == '\n' || byteVal == '\r') {
        rxPacketState = RxPacketState::WAIT_CRC;
        return;
    }

    // 3. Вывод символа (ASCII + UTF-8 Русские буквы)
    if (byteVal >= 32 && byteVal != 127) {
        Serial.write(byteVal);

        if (rxBufferIndex < Config::RX_BUFFER_SIZE - 1) {
            rxSentenceBuffer[rxBufferIndex++] = static_cast<char>(byteVal);
            rxSentenceBuffer[rxBufferIndex] = '\0';
        }
    }
}

// ============================================================================
// ИТОГОВАЯ ОБРАБОТКА ВХОДЯЩЕГО СООБЩЕНИЯ
// ============================================================================
void finalizeReceivedMessage(bool hasCRC, uint8_t receivedCRC) {
    rxIsReceivingMessage = false;
    rxPacketState = RxPacketState::PAYLOAD;

    if (rxBufferIndex == 0) return;

    uint32_t totalDurationMs = rxLastCharTimeMs - rxMessageStartTimeMs + 350;
    if (totalDurationMs < 100) totalDurationMs = 100;
    float durationSec = totalDurationMs / 1000.0f;

    float bytesPerSec = static_cast<float>(rxBufferIndex) / durationSec;
    float bitsPerSec = bytesPerSec * 8.0f;

    int avgLightAdc = (lightSamplesCount > 0) ? static_cast<int>(totalLightAdcSum / lightSamplesCount) : peakLightAdc;
    int contrastDelta = avgLightAdc - ambientNoiseLevel;

    float snrDb = 0.0f;
    int noiseBase = max(1, ambientNoiseLevel);
    if (avgLightAdc > noiseBase) {
        snrDb = 20.0f * log10(static_cast<float>(avgLightAdc) / static_cast<float>(noiseBase));
    }

    uint8_t calculatedCRC = calculateCRC8(reinterpret_cast<const uint8_t*>(rxSentenceBuffer), rxBufferIndex);

    Serial.println();
    Serial.println(F("\n************************************************************"));
    Serial.print(F(">>> [RX: ПРИНЯТОЕ СООБЩЕНИЕ]: \""));
    Serial.print(rxSentenceBuffer);
    Serial.println(F("\""));
    Serial.print(F(">>> [РАЗМЕР СООБЩЕНИЯ]:       "));
    Serial.print(rxBufferIndex);
    Serial.println(F(" байт"));

    bool isCrcValid = false;

    if (hasCRC) {
        Serial.print(F(">>> [КОНТРОЛЬ CRC-8]:          Расчетный = 0x"));
        if (calculatedCRC < 16) Serial.print(F("0"));
        Serial.print(calculatedCRC, HEX);
        Serial.print(F(" | Принятый = 0x"));
        if (receivedCRC < 16) Serial.print(F("0"));
        Serial.println(receivedCRC, HEX);

        if (calculatedCRC == receivedCRC) {
            Serial.println(F(">>> [СТАТУС ЦЕЛОСТНОСТИ]:      [УСПЕШНО - ОШИБОК НЕТ!] ✔"));
            isCrcValid = true;
        } else {
            Serial.println(F(">>> [СТАТУС ЦЕЛОСТНОСТИ]:      [ОШИБКА CRC! ДАННЫЕ ИСКАЖЕНЫ] ❌"));
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
    Serial.print(F("    • Оптический контраст (ΔV): "));
    Serial.print(contrastDelta);
    Serial.print(F(" ADC (Луч: "));
    Serial.print(avgLightAdc);
    Serial.print(F(" | Фон: "));
    Serial.print(ambientNoiseLevel);
    Serial.println(F(")"));

    Serial.print(F("    • SNR (Сигнал/Шум):         "));
    Serial.print(snrDb, 1);
    Serial.print(F(" dB "));
    if (snrDb >= 25.0f) {
        Serial.println(F("[ОТЛИЧНЫЙ СИГНАЛ] ★★★"));
    } else if (snrDb >= 16.0f) {
        Serial.println(F("[ХОРОШИЙ СИГНАЛ] ★★☆"));
    } else {
        Serial.println(F("[СЛАБЫЙ СИГНАЛ] ☆☆☆"));
    }

    Serial.print(F("    • Скорость передачи:        "));
    Serial.print(bytesPerSec, 1);
    Serial.print(F(" байт/с ("));
    Serial.print(bitsPerSec, 1);
    Serial.println(F(" бит/с)"));
    Serial.println(F("************************************************************\n"));

    // ОТВЕТНЫЙ СИГНАЛ ПО ПРОТОКОЛУ ARQ
    if (isCrcValid) {
        // Отправляем оптический ACK обратно отправителю
        sendAckFrame(Config::CTRL_ACK);
        Sound::playRxReceived();
    } else {
        // Отправляем оптический NAK для запроса автоповтора
        sendAckFrame(Config::CTRL_NAK);
        Sound::playCrcError();
    }

    rxBufferIndex = 0;
    rxSentenceBuffer[0] = '\0';
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

    Serial.print(F("[КАЛИБРОВКА] Фоновая темнота: "));
    Serial.print(ambientNoiseLevel);
    Serial.println(F(" (ADC 0..1023)\n"));
}
