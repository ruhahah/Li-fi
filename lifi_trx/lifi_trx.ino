/*
 * ============================================================================
 * ПРОЕКТ: Li-Fi Двусторонняя полудуплексная связь (Visible Light Communication)
 * МОДУЛЬ: ТРАНСИВЕР (TRANSCEIVER / HALF-DUPLEX)
 * ПЛАТФОРМА: Arduino Uno (ATmega328P)
 * 
 * АППАРАТНАЯ КОНФИГУРАЦИЯ:
 *   - TX (Передатчик):  LED на Pin 13 (через резистор 150-220 Ом на GND)
 *   - RX (Приемник):    Фотодиод BPW24 на Pin A0 (Катод -> 5V, Анод -> A0, 5-10k -> GND)
 *   - Связь с ПК:       Hardware Serial UART (115200 baud)
 * 
 * ПАРАМЕТРЫ ПРОТОКОЛА:
 *   - Скорость:         30 бод (длительность 1 бита = 33 333 мкс)
 *   - Модуляция:        OOK (On-Off Keying)
 *   - Формат кадра:     8-N-1 (1 Start HIGH, 8 Data LSB first, 1 Stop LOW, 1 Guard LOW)
 *   - Синхронизация:    Абсолютная микросекундная (micros()), без блокирующих delay()
 *   - Кодировка:        Прозрачная UTF-8 (Кириллица + Латиница)
 *   - Целостность:      Контрольная сумма CRC-8 (Полином 0x07: x^8 + x^2 + x + 1)
 * ============================================================================
 */

#include <Arduino.h>

// ============================================================================
// КОНФИГУРАЦИЯ И АППАРАТНЫЕ ПАРАМЕТРЫ
// ============================================================================
namespace Config {
    constexpr uint8_t PIN_TX = 13;                       // Пин оптического передатчика (LED)
    constexpr uint8_t PIN_RX = A0;                       // Пин оптического приемника (Фотодиод)

    constexpr uint16_t BAUD_RATE = 30;                   // Скорость оптического канала: 30 бит/с
    constexpr uint32_t BIT_PERIOD_US = 1000000UL / BAUD_RATE; // 33 333 мкс (1 бит)
    constexpr uint32_t INTER_BYTE_GUARD_US = BIT_PERIOD_US;   // 33 333 мкс (пауза между байтами)

    constexpr size_t MAX_BUFFER_SIZE = 512;              // Размер буфера сообщений
    constexpr uint32_t MESSAGE_TIMEOUT_MS = 500;         // Таймаут тишины завершения приема

    constexpr int TRIGGER_THRESHOLD_MARGIN = 12;         // Превышение над шумом для старт-триггера
    constexpr int MIN_SIGNAL_DELTA = 15;                 // Минимальная амплитуда полезного сигнала
    constexpr int32_t VOTING_OFFSETS_US[3] = {-2000, 0, 2000}; // Сдвиги для 3-точечного оверсэмплирования
}

// ============================================================================
// СОСТОЯНИЯ КОНЕЧНОГО АВТОМАТА (STATE MACHINE)
// ============================================================================
enum class TransceiverState : uint8_t {
    IDLE_LISTENING,     // Режим слушателя по умолчанию (оцифровка A0)
    RECEIVING,          // Активный прием оптического пакета
    TRANSMITTING        // Активная передача в оптический канал
};

// ============================================================================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ МОДУЛЯ
// ============================================================================
TransceiverState currentState = TransceiverState::IDLE_LISTENING;

// Буфер полезной нагрузки
char rxBuffer[Config::MAX_BUFFER_SIZE];
size_t rxBufferIndex = 0;

// Временные метки
uint32_t lastCharTimeMs = 0;
uint32_t messageStartTimeMs = 0;

// Параметры адаптивного оптического канала
int ambientNoiseLevel = 3;                               // Уровень фоновой темноты
int dynamicThreshold = 40;                              // Расчетный порог (середина между темнотой и лучом)
int peakLightLevel = 150;                               // Замеренный пик света старт-бита
long totalLightAdcSum = 0;                              // Накопитель яркости для расчета SNR
int lightSamplesCount = 0;

// Состояние протокола приема
bool waitingForCRC = false;

// ============================================================================
// ПРОТОТИПЫ ФУНКЦИЙ
// ============================================================================
uint8_t calculateCRC8(const uint8_t* data, size_t len);
void calibrateAmbientLight();
bool checkStartBitTrigger();
bool sampleBitWithVoting(uint32_t bitCenterUs, int threshold);
bool receiveByte(uint8_t& outByte);
void processIncomingByte(uint8_t byteVal);
void finalizeReceivedPacket(bool hasCRC, uint8_t receivedCRC);

void sendBit(bool bitVal);
void sendByte(uint8_t data);
void transmitPacket(const uint8_t* payload, size_t len);
void handleSerialCommands();

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    pinMode(Config::PIN_TX, OUTPUT);
    digitalWrite(Config::PIN_TX, LOW); // Линия передачи в покое (LED выключен)

    pinMode(Config::PIN_RX, INPUT);    // Вход фотодиода

    // Запуск аппаратного UART для связи с ПК
    Serial.begin(115200);
    delay(500); // Фаза стабилизации питания и подключения терминала

    Serial.println(F("\n============================================================"));
    Serial.println(F("   >>> Li-Fi ПОЛУДУПЛЕКСНЫЙ ТРАНСИВЕР (HALF-DUPLEX) <<<     "));
    Serial.println(F("============================================================"));
    Serial.print(F("[CONFIG] Скорость Li-Fi:       "));
    Serial.print(Config::BAUD_RATE);
    Serial.print(F(" бод (бит/с) | Период бита: "));
    Serial.print(Config::BIT_PERIOD_US / 1000);
    Serial.println(F(" мс"));
    Serial.println(F("[CONFIG] Режим работы:         Полудуплекс (Push-to-Talk / Рация)"));
    Serial.println(F("[CONFIG] Кодировка:            UTF-8 (Русский + English)"));
    Serial.println(F("[CONFIG] Контроль ошибок:      CRC-8 (Полином 0x07)"));
    Serial.println(F("[CONFIG] Синхронизация:        Прецизионная micros() + 3X Voting"));

    // Калибровка фонового освещения в помещении
    calibrateAmbientLight();

    Serial.println(F("------------------------------------------------------------"));
    Serial.println(F("СИСТЕМА ГОТОВА:"));
    Serial.println(F("  - Для отправки: введите текст в Serial Monitor и нажмите Enter"));
    Serial.println(F("  - Для калибровки темноты: отправьте 'c'"));
    Serial.println(F("  - Для замера АЦП: отправьте 'r'"));
    Serial.println(F("------------------------------------------------------------\n"));
}

// ============================================================================
// ГЛАВНЫЙ ЦИКЛ (LOOP / STATE MACHINE)
// ============================================================================
void loop() {
    // ------------------------------------------------------------------------
    // 1. ПРИОРИТЕТ: ПЕРЕХОД В РЕЖИМ ПЕРЕДАЧИ (TX) ПРИ НАЛИЧИИ ВВОДА С ПК
    // ------------------------------------------------------------------------
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.length() > 0) {
            // Перехват служебных команд
            if (input.length() == 1) {
                char cmd = input.charAt(0);
                if (cmd == 'c' || cmd == 'C') {
                    calibrateAmbientLight();
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

            // Переключение автомата в режим передачи (TX)
            currentState = TransceiverState::TRANSMITTING;

            // Передача пакета в оптический канал
            transmitPacket(reinterpret_cast<const uint8_t*>(input.c_str()), input.length());

            // Возврат в режим слушателя (RX)
            currentState = TransceiverState::IDLE_LISTENING;
            return;
        }
    }

    // ------------------------------------------------------------------------
    // 2. РЕЖИМ СЛУШАТЕЛЯ (RX): ОЦИФРОВКА A0 И ДЕТЕКЦИЯ СТАРТ-БИТА
    // ------------------------------------------------------------------------
    if (currentState == TransceiverState::IDLE_LISTENING) {
        if (checkStartBitTrigger()) {
            uint8_t receivedByte = 0;

            // Попытка приема байта
            if (receiveByte(receivedByte)) {
                processIncomingByte(receivedByte);
            }
        }

        // Завершение приема по таймауту тишины
        if (rxBufferIndex > 0 && (millis() - lastCharTimeMs > Config::MESSAGE_TIMEOUT_MS)) {
            finalizeReceivedPacket(false, 0);
        }
    }
}

// ============================================================================
// РАСЧЕТ КОНТРОЛЬНОЙ СУММЫ CRC-8
// ============================================================================
/**
 * @brief Вычисление контрольной суммы CRC-8 (Полином x^8 + x^2 + x + 1, 0x07)
 * @param data Указатель на массив байтов
 * @param len Длина массива в байтах
 * @return 8-битная контрольная сумма
 */
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
// ПЕРЕДАТЧИК (TRANSMITTER / TX)
// ============================================================================

/**
 * @brief Прецизионная передача одного оптического бита через micros()
 */
void sendBit(bool bitVal) {
    uint32_t startTimeUs = micros();
    digitalWrite(Config::PIN_TX, bitVal ? HIGH : LOW);
    
    // Блокирующее микросекундное ожидание для стабильности фазы
    while ((long)(micros() - (startTimeUs + Config::BIT_PERIOD_US)) < 0) {
        // NOP
    }
}

/**
 * @brief Передача одного байта по протоколу 8-N-1 + Защитная пауза
 */
void sendByte(uint8_t data) {
    uint32_t frameStartUs = micros();

    // 1. СТАРТОВЫЙ БИТ (HIGH)
    digitalWrite(Config::PIN_TX, HIGH);
    while ((long)(micros() - (frameStartUs + Config::BIT_PERIOD_US)) < 0);

    // 2. 8 БИТ ДАННЫХ (LSB first)
    for (uint8_t i = 0; i < 8; i++) {
        bool bit = (data >> i) & 0x01;
        digitalWrite(Config::PIN_TX, bit ? HIGH : LOW);
        
        uint32_t targetUs = frameStartUs + ((uint32_t)(i + 2) * Config::BIT_PERIOD_US);
        while ((long)(micros() - targetUs) < 0);
    }

    // 3. СТОПОВЫЙ БИТ (LOW)
    digitalWrite(Config::PIN_TX, LOW);
    uint32_t stopTargetUs = frameStartUs + (10UL * Config::BIT_PERIOD_US);
    while ((long)(micros() - stopTargetUs) < 0);

    // 4. МЕЖСИМВОЛЬНАЯ ЗАЩИТНАЯ ПАУЗА (LOW)
    uint32_t guardTargetUs = stopTargetUs + Config::INTER_BYTE_GUARD_US;
    while ((long)(micros() - guardTargetUs) < 0);
}

/**
 * @brief Полный цикл передачи пакета в оптический канал
 */
void transmitPacket(const uint8_t* payload, size_t len) {
    uint8_t crc = calculateCRC8(payload, len);

    Serial.println(F("\n>>>>>>>>>>>>>> [ TX: НАЧАЛО ПЕРЕДАЧИ ] >>>>>>>>>>>>>>"));
    Serial.print(F("[TX] Сообщение: \""));
    for (size_t i = 0; i < len; i++) {
        Serial.write(payload[i]);
    }
    Serial.println(F("\""));
    Serial.print(F("[TX] Объем:     "));
    Serial.print(len);
    Serial.print(F(" байт | CRC-8: 0x"));
    if (crc < 16) Serial.print(F("0"));
    Serial.println(crc, HEX);
    Serial.println(F("-----------------------------------------------------"));

    // 1. Передача полезной нагрузки (символы UTF-8)
    for (size_t i = 0; i < len; i++) {
        sendByte(payload[i]);
    }

    // 2. Передача маркера окончания текста
    sendByte('\n');

    // 3. Передача контрольного байта CRC-8
    sendByte(crc);

    Serial.println(F("-----------------------------------------------------"));
    Serial.println(F("<<<<<<<<<<<<<< [ TX: ПЕРЕДАЧА ЗАВЕРШЕНА ] <<<<<<<<<<<<<<\n"));
}

// ============================================================================
// ПРИЕМНИК (RECEIVER / RX)
// ============================================================================

/**
 * @brief Проверка превышения напряжения для детекции переднего фронта старт-бита
 */
bool checkStartBitTrigger() {
    int val = analogRead(Config::PIN_RX);
    return (val > (ambientNoiseLevel + Config::TRIGGER_THRESHOLD_MARGIN));
}

/**
 * @brief 3-кратный оверсэмплинг со сдвигом (-2000 мкс, 0 мкс, +2000 мкс)
 */
bool sampleBitWithVoting(uint32_t bitCenterUs, int threshold) {
    int highVotes = 0;

    for (int i = 0; i < 3; i++) {
        uint32_t sampleTimeUs = bitCenterUs + Config::VOTING_OFFSETS_US[i];
        while ((long)(micros() - sampleTimeUs) < 0) {
            // Прецизионное ожидание
        }
        
        int val = analogRead(Config::PIN_RX);
        if (val >= threshold) {
            highVotes++;
        }
    }

    // Мажоритарное решение: 2 из 3 голосов
    return (highVotes >= 2);
}

/**
 * @brief Прием байта с динамической адаптацией порога по амплитуде старт-бита
 */
bool receiveByte(uint8_t& outByte) {
    uint32_t frameStartUs = micros();

    // Шаг 1: Центр стартового бита (+ 0.5 * T = 16.6 мс)
    uint32_t startCenterUs = frameStartUs + (Config::BIT_PERIOD_US / 2);
    while ((long)(micros() - startCenterUs) < 0);

    int startSample = analogRead(Config::PIN_RX);
    if (startSample <= (ambientNoiseLevel + Config::MIN_SIGNAL_DELTA)) {
        return false; // Ложная оптическая помеха
    }

    // ДИНАМИЧЕСКИЙ РАСЧЕТ ПОРОГА: Середина между фоновым шумом и пиком старт-бита
    peakLightLevel = startSample;
    dynamicThreshold = (ambientNoiseLevel + peakLightLevel) / 2;

    // Сбор статистики для SNR
    totalLightAdcSum += peakLightLevel;
    lightSamplesCount++;

    uint8_t reconstructedByte = 0;

    // Шаг 2: Считывание 8 бит данных строго в их геометрических центрах
    for (uint8_t bitIdx = 0; bitIdx < 8; bitIdx++) {
        uint32_t bitCenterUs = frameStartUs + (Config::BIT_PERIOD_US * 3 / 2) + ((uint32_t)bitIdx * Config::BIT_PERIOD_US);

        bool bitVal = sampleBitWithVoting(bitCenterUs, dynamicThreshold);
        if (bitVal) {
            reconstructedByte |= (1 << bitIdx);
        }
    }

    // Шаг 3: Проверка стопового бита (+ 9.5 * T)
    uint32_t stopCenterUs = frameStartUs + (Config::BIT_PERIOD_US * 19 / 2);
    while ((long)(micros() - stopCenterUs) < 0);

    int stopAdc = analogRead(Config::PIN_RX);
    bool isStopValid = (stopAdc < dynamicThreshold);

    // Дожидаемся полного окончания кадра (+ 10.0 * T)
    uint32_t frameEndUs = frameStartUs + (Config::BIT_PERIOD_US * 10);
    while ((long)(micros() - frameEndUs) < 0);

    if (isStopValid) {
        outByte = reconstructedByte;
        return true;
    }

    return false;
}

/**
 * @brief Обработка и сохранение принятого байта в буфер
 */
void processIncomingByte(uint8_t byteVal) {
    if (rxBufferIndex == 0 && !waitingForCRC) {
        messageStartTimeMs = millis();
        totalLightAdcSum = 0;
        lightSamplesCount = 0;
    }
    lastCharTimeMs = millis();

    // 1. Если ожидается контрольный байт CRC-8
    if (waitingForCRC) {
        finalizeReceivedPacket(true, byteVal);
        return;
    }

    // 2. Маркер окончания текста -> переключаемся в ожидание CRC-8
    if (byteVal == '\n' || byteVal == '\r') {
        waitingForCRC = true;
        return;
    }

    // 3. Сохранение печатных символов (ASCII + UTF-8)
    if (byteVal >= 32 && byteVal != 127) {
        Serial.write(byteVal); // Мгновенный прозрачный вывод в монитор порта

        if (rxBufferIndex < Config::MAX_BUFFER_SIZE - 1) {
            rxBuffer[rxBufferIndex++] = static_cast<char>(byteVal);
            rxBuffer[rxBufferIndex] = '\0';
        }
    }
}

/**
 * @brief Итоговая обработка принятого пакета: проверка CRC-8 и вывод телеметрии
 */
void finalizeReceivedPacket(bool hasCRC, uint8_t receivedCRC) {
    waitingForCRC = false;

    if (rxBufferIndex == 0) return;

    // Расчет времени передачи
    uint32_t totalDurationMs = lastCharTimeMs - messageStartTimeMs + 350;
    if (totalDurationMs < 100) totalDurationMs = 100;
    float durationSec = totalDurationMs / 1000.0f;

    // Расчет скорости
    float bytesPerSec = static_cast<float>(rxBufferIndex) / durationSec;
    float bitsPerSec = bytesPerSec * 8.0f;

    // Расчет параметров оптического канала
    int avgLightAdc = (lightSamplesCount > 0) ? static_cast<int>(totalLightAdcSum / lightSamplesCount) : peakLightLevel;
    int contrastDelta = avgLightAdc - ambientNoiseLevel;

    // Расчет SNR (Отношение сигнал/шум в дБ)
    float snrDb = 0.0f;
    int noiseBase = max(1, ambientNoiseLevel);
    if (avgLightAdc > noiseBase) {
        snrDb = 20.0f * log10(static_cast<float>(avgLightAdc) / static_cast<float>(noiseBase));
    }

    // Расчет локальной контрольной суммы CRC-8
    uint8_t calculatedCRC = calculateCRC8(reinterpret_cast<const uint8_t*>(rxBuffer), rxBufferIndex);

    Serial.println();
    Serial.println(F("\n************************************************************"));
    Serial.print(F(">>> [RX: ПРИНЯТОЕ СООБЩЕНИЕ]: \""));
    Serial.print(rxBuffer);
    Serial.println(F("\""));
    Serial.print(F(">>> [РАЗМЕР СООБЩЕНИЯ]:       "));
    Serial.print(rxBufferIndex);
    Serial.println(F(" байт"));

    // Проверка контрольной суммы
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
        Serial.print(F(">>> [КОНТРОЛЬ CRC-8]:          Таймаут ожидания CRC (Расчет: 0x"));
        if (calculatedCRC < 16) Serial.print(F("0"));
        Serial.print(calculatedCRC, HEX);
        Serial.println(F(")"));
    }

    // Вывод телеметрии оптического канала
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

    Serial.print(F("    • Скорость передачи данных: "));
    Serial.print(bytesPerSec, 1);
    Serial.print(F(" байт/с ("));
    Serial.print(bitsPerSec, 1);
    Serial.println(F(" бит/с)"));

    Serial.print(F("    • Время сеанса связи:       "));
    Serial.print(durationSec, 1);
    Serial.println(F(" сек"));
    Serial.println(F("************************************************************\n"));

    // Очистка буфера для следующего приема
    rxBufferIndex = 0;
    rxBuffer[0] = '\0';
}

// ============================================================================
// КАЛИБРОВКА ОСВЕЩЕНИЯ
// ============================================================================

void calibrateAmbientLight() {
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

    Serial.print(F("[КАЛИБРОВКА] Фоновый шум темноты: "));
    Serial.print(ambientNoiseLevel);
    Serial.println(F(" (ADC 0..1023)\n"));
}
