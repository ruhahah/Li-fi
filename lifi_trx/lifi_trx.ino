/*
 * ============================================================================
 * ПРОЕКТ: Li-Fi Полнодуплексная оптическая связь (Visible Light Communication)
 * МОДУЛЬ: ТРАНСИВЕР (FULL-DUPLEX / СИМУЛЬТАННАЯ ПЕРЕДАЧА И ПРИЕМ)
 * ПЛАТФОРМА: Arduino Uno (ATmega328P)
 * 
 * АППАРАТНАЯ КОНФИГУРАЦИЯ:
 *   - TX (Передатчик):  LED на Pin 13 (через резистор 150-220 Ом на GND)
 *   - RX (Приемник):    Фотодиод BPW24 на Pin A0 (Катод -> 5V, Анод -> A0, Резистор -> GND)
 *   - Связь с ПК:       Hardware Serial UART (115200 baud)
 * 
 * ОСОБЕННОСТИ АРХИТЕКТУРЫ FULL-DUPLEX:
 *   - 100% НЕБЛОКИРУЮЩИЕ асинхронные конечные автоматы для TX и RX.
 *   - Полное отсутствие delay() и блокирующих while() в основном цикле.
 *   - Обе платы могут ОДНОВРЕМЕННО передавать и принимать в одну и ту же миллисекунду.
 *   - Динамическая адаптация порога на лету + 3-точечное мажоритарное голосование.
 *   - Поддержка Русского (UTF-8) и Английского языков + аппаратный CRC-8.
 * ============================================================================
 */

#include <Arduino.h>

// ============================================================================
// КОНФИГУРАЦИЯ И КОНСТАНТЫ
// ============================================================================
namespace Config {
    constexpr uint8_t PIN_TX = 13;                         // Пин светодиода (TX)
    constexpr uint8_t PIN_RX = A0;                         // Пин фотодиода (RX)

    constexpr uint16_t BAUD_RATE = 30;                     // Скорость: 30 бит/с
    constexpr uint32_t BIT_PERIOD_US = 1000000UL / BAUD_RATE; // 33 333 мкс
    constexpr uint32_t GUARD_PERIOD_US = BIT_PERIOD_US;    // 33 333 мкс (пауза между байтами)

    constexpr size_t TX_QUEUE_SIZE = 256;                  // Очередь отправки TX
    constexpr size_t RX_BUFFER_SIZE = 256;                 // Буфер приема RX
    constexpr uint32_t MESSAGE_TIMEOUT_MS = 500;           // Таймаут завершения фразы

    constexpr int TRIGGER_MARGIN = 12;                     // Порог старт-триггера над шумом
    constexpr int MIN_SIGNAL_DELTA = 15;                   // Минимальная амплитуда луча
    constexpr int32_t VOTING_OFFSETS_US[3] = {-2000, 0, 2000}; // Сдвиги для 3X оверсэмплирования
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
};

TxRingBuffer txQueue;

// ============================================================================
// СОСТОЯНИЯ АВТОМАТОВ TX И RX
// ============================================================================
enum class TxState : uint8_t {
    IDLE,               // Передатчик свободен
    START_BIT,          // Передача старт-бита (HIGH)
    DATA_BITS,          // Передача 8 бит данных (LSB first)
    STOP_BIT,           // Передача стоп-бита (LOW)
    GUARD_PAUSE         // Межсимвольная защитная пауза (LOW)
};

enum class RxState : uint8_t {
    IDLE_WAIT_FRONT,    // Ожидание переднего фронта старт-бита
    VERIFY_START_BIT,   // Проверка центра старт-бита
    SAMPLE_DATA_BITS,   // 3-точечное сэмплирование 8 бит
    VERIFY_STOP_BIT,    // Проверка стоп-бита
    COMPLETE_FRAME      // Завершение кадра
};

enum class RxPacketState : uint8_t {
    PAYLOAD,            // Прием текста
    WAIT_CRC            // Ожидание байта CRC-8
};

// ============================================================================
// ПЕРЕМЕННЫЕ ПЕРЕДАТЧИКА (TX ENGINE)
// ============================================================================
TxState txState = TxState::IDLE;
uint32_t txBitDeadlineUs = 0;
uint8_t txCurrentByte = 0;
uint8_t txBitIndex = 0;

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

int ambientNoiseLevel = 3;                                 // Фоновый уровень темноты
int dynamicThreshold = 35;                                // Адаптивный порог
int peakLightAdc = 150;                                   // Пиковая яркость луча
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
void calibrateDarkness();
void processReceivedByte(uint8_t byteVal);
void finalizeReceivedMessage(bool hasCRC, uint8_t receivedCRC);

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    pinMode(Config::PIN_TX, OUTPUT);
    digitalWrite(Config::PIN_TX, LOW); // Светодиод выключен в покое

    pinMode(Config::PIN_RX, INPUT);

    Serial.begin(115200);
    delay(500);

    Serial.println(F("\n============================================================"));
    Serial.println(F("    >>> Li-Fi ПОЛНОДУПЛЕКСНЫЙ ТРАНСИВЕР (FULL-DUPLEX) <<<   "));
    Serial.println(F("============================================================"));
    Serial.print(F("[INFO] Скорость канала:       "));
    Serial.print(Config::BAUD_RATE);
    Serial.print(F(" бод (бит/с) | Бит: "));
    Serial.print(Config::BIT_PERIOD_US / 1000);
    Serial.println(F(" мс"));
    Serial.println(F("[INFO] Режим связи:           FULL-DUPLEX (Одновременный TX и RX)"));
    Serial.println(F("[INFO] Асинхронный движок:    Неблокирующий (Zero CPU Delay)"));
    Serial.println(F("[INFO] Поддержка языков:      UTF-8 Русский + English + CRC-8"));

    calibrateDarkness();

    Serial.println(F("------------------------------------------------------------"));
    Serial.println(F("Трансивер готов к одновременной двусторонней связи!\n"));
}

// ============================================================================
// ГЛАВНЫЙ ЦИКЛ (НЕБЛОКИРУЮЩИЙ СИНХРОННЫЙ ДИСПЕТЧЕР)
// ============================================================================
void loop() {
    // 1. Опрос ввода с ПК (добавление исходящих сообщений в очередь TX)
    handleSerialInput();

    // 2. Шаг автомата передатчика (TX)
    updateTxEngine();

    // 3. Шаг автомата приемника (RX)
    updateRxEngine();

    // 4. Проверка таймаута тишины для завершения входящего сообщения
    if (rxIsReceivingMessage && (millis() - rxLastCharTimeMs > Config::MESSAGE_TIMEOUT_MS)) {
        finalizeReceivedMessage(false, 0);
    }
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
// ОБРАБОТКА ВВОДА СЕРИЙНОГО ПОРТА (ПК -> TX QUEUE)
// ============================================================================
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

            // Вычисляем CRC-8 от исходящего сообщения
            uint8_t crc = calculateCRC8(reinterpret_cast<const uint8_t*>(input.c_str()), input.length());

            Serial.println(F("\n>>>>>>>>>>>>>> [ TX: ОТПРАВКА СООБЩЕНИЯ ] >>>>>>>>>>>>>>"));
            Serial.print(F("[TX Текст]: \""));
            Serial.print(input);
            Serial.println(F("\""));
            Serial.print(F("[TX Объем]: "));
            Serial.print(input.length());
            Serial.print(F(" байт | CRC-8: 0x"));
            if (crc < 16) Serial.print(F("0"));
            Serial.println(crc, HEX);
            Serial.println(F("--------------------------------------------------------"));

            // 1. Помещаем байты текста в очередь отправки
            for (size_t i = 0; i < input.length(); i++) {
                txQueue.push(static_cast<uint8_t>(input.charAt(i)));
            }

            // 2. Добавляем маркер окончания текста ('\n')
            txQueue.push('\n');

            // 3. Добавляем контрольный байт CRC-8
            txQueue.push(crc);
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
                    // Старт передачи байта: выставляем СТАРТ-БИТ (HIGH)
                    digitalWrite(Config::PIN_TX, HIGH);
                    txBitDeadlineUs = currentUs + Config::BIT_PERIOD_US;
                    txState = TxState::START_BIT;
                }
            }
            break;

        case TxState::START_BIT:
            if ((long)(currentUs - txBitDeadlineUs) >= 0) {
                // Переход к первому биту данных (Bit 0)
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
                    // Все 8 бит переданы -> выставляем СТОП-БИТ (LOW)
                    digitalWrite(Config::PIN_TX, LOW);
                    txBitDeadlineUs += Config::BIT_PERIOD_US;
                    txState = TxState::STOP_BIT;
                }
            }
            break;

        case TxState::STOP_BIT:
            if ((long)(currentUs - txBitDeadlineUs) >= 0) {
                // Переход к межсимвольной защитной паузе (LOW)
                digitalWrite(Config::PIN_TX, LOW);
                txBitDeadlineUs += Config::GUARD_PERIOD_US;
                txState = TxState::GUARD_PAUSE;
            }
            break;

        case TxState::GUARD_PAUSE:
            if ((long)(currentUs - txBitDeadlineUs) >= 0) {
                // Байт полностью передан
                if (txQueue.isEmpty()) {
                    txState = TxState::IDLE;
                    Serial.println(F("[TX]: Вся очередь успешно отправлена в луч!\n"));
                } else {
                    // Сразу начинаем следующий байт из очереди
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
        // --------------------------------------------------------------------
        // 1. Ожидание оптического фронта старт-бита
        // --------------------------------------------------------------------
        case RxState::IDLE_WAIT_FRONT: {
            int val = analogRead(Config::PIN_RX);
            if (val > (ambientNoiseLevel + Config::TRIGGER_MARGIN)) {
                rxFrameStartUs = currentUs;
                rxState = RxState::VERIFY_START_BIT;
            }
            break;
        }

        // --------------------------------------------------------------------
        // 2. Валидация центра старт-бита (+0.5 T = 16.6 мс)
        // --------------------------------------------------------------------
        case RxState::VERIFY_START_BIT: {
            uint32_t targetUs = rxFrameStartUs + (Config::BIT_PERIOD_US / 2);
            if ((long)(currentUs - targetUs) >= 0) {
                int sample = analogRead(Config::PIN_RX);
                if (sample <= (ambientNoiseLevel + Config::MIN_SIGNAL_DELTA)) {
                    // Ложная оптическая помеха
                    rxState = RxState::IDLE_WAIT_FRONT;
                } else {
                    // Динамическая адаптация порога на лету
                    peakLightAdc = sample;
                    dynamicThreshold = (ambientNoiseLevel + peakLightAdc) / 2;
                    hysteresisVal = max(4, (peakLightAdc - ambientNoiseLevel) / 10);

                    totalLightAdcSum += peakLightAdc;
                    lightSamplesCount++;

                    // Инициализация приема 8 бит данных
                    rxBitIndex = 0;
                    rxReconstructedByte = 0;
                    rxVotingIndex = 0;
                    rxHighVotesCount = 0;
                    rxState = RxState::SAMPLE_DATA_BITS;
                }
            }
            break;
        }

        // --------------------------------------------------------------------
        // 3. Считывание 8 бит данных с 3-кратным оверсэмплированием
        // --------------------------------------------------------------------
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
                    // 3 замера сделаны: принимаем решение большинством голосов (2 из 3)
                    if (rxHighVotesCount >= 2) {
                        rxReconstructedByte |= (1 << rxBitIndex);
                    }

                    // Переходим к следующему биту
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

        // --------------------------------------------------------------------
        // 4. Проверка стоп-бита (+9.5 T)
        // --------------------------------------------------------------------
        case RxState::VERIFY_STOP_BIT: {
            uint32_t stopTargetUs = rxFrameStartUs + (Config::BIT_PERIOD_US * 19 / 2);
            if ((long)(currentUs - stopTargetUs) >= 0) {
                int stopAdc = analogRead(Config::PIN_RX);
                bool isStopValid = (stopAdc < dynamicThreshold);

                if (isStopValid) {
                    rxState = RxState::COMPLETE_FRAME;
                } else {
                    // Ошибка стоп-бита
                    rxState = RxState::IDLE_WAIT_FRONT;
                }
            }
            break;
        }

        // --------------------------------------------------------------------
        // 5. Завершение кадра (+10.0 T) и передача байта на обработку
        // --------------------------------------------------------------------
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
    if (!rxIsReceivingMessage) {
        rxIsReceivingMessage = true;
        rxMessageStartTimeMs = millis();
        totalLightAdcSum = 0;
        lightSamplesCount = 0;
    }
    rxLastCharTimeMs = millis();

    // 1. Прием контрольного байта CRC-8
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
        Serial.write(byteVal); // Прозрачный вывод

        if (rxBufferIndex < Config::RX_BUFFER_SIZE - 1) {
            rxSentenceBuffer[rxBufferIndex++] = static_cast<char>(byteVal);
            rxSentenceBuffer[rxBufferIndex] = '\0';
        }
    }
}

// ============================================================================
// ИТОГОВАЯ ОБРАБОТКА ВХОДЯЩЕГО СООБЩЕНИЯ (CRC-8 + ТЕЛЕМЕТРИЯ)
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

    if (hasCRC) {
        Serial.print(F(">>> [КОНТРОЛЬ CRC-8]:          Расчетный = 0x"));
        if (calculatedCRC < 16) Serial.print(F("0"));
        Serial.print(calculatedCRC, HEX);
        Serial.print(F(" | Принятый = 0x"));
        if (receivedCRC < 16) Serial.print(F("0"));
        Serial.println(receivedCRC, HEX);

        if (calculatedCRC == receivedCRC) {
            Serial.println(F(">>> [СТАТУС ЦЕЛОСТНОСТИ]:      [УСПЕШНО - ОШИБОК НЕТ!] ✔"));
        } else {
            Serial.println(F(">>> [СТАТУС ЦЕЛОСТНОСТИ]:      [ОШИБКА CRC! ДАННЫЕ ИСКАЖЕНЫ] ❌"));
        }
    } else {
        Serial.print(F(">>> [КОНТРОЛЬ CRC-8]:          Таймаут CRC (Расчет: 0x"));
        if (calculatedCRC < 16) Serial.print(F("0"));
        Serial.print(calculatedCRC, HEX);
        Serial.println(F(")"));
    }

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
    } else if (snrDb >= 8.0f) {
        Serial.println(F("[УДОВЛЕТВОРИТЕЛЬНО] ★☆☆"));
    } else {
        Serial.println(F("[СЛАБЫЙ СИГНАЛ / ШУМ] ☆☆☆"));
    }

    Serial.print(F("    • Скорость передачи:        "));
    Serial.print(bytesPerSec, 1);
    Serial.print(F(" байт/с ("));
    Serial.print(bitsPerSec, 1);
    Serial.println(F(" бит/с)"));
    Serial.println(F("************************************************************\n"));

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
