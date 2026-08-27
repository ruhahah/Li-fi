/*
 * ============================================================================
 * ПРОЕКТ: Li-Fi Односторонняя оптическая связь (Visible Light Communication)
 * МОДУЛЬ: ПРИЕМНИК (RX) - АВТОАДАПТАЦИЯ + КИРИЛЛИЦА + CRC-8 + ТЕЛЕМЕТРИЯ SNR
 * ПЛАТФОРМА: Arduino Uno (ATmega328P)
 * ДАТЧИК: Фотодиод BPW24 (Катод -> 5V, Анод -> A0, Резистор -> GND)
 * ============================================================================
 */

#include <Arduino.h>

// ==========================================
// КОНФИГУРАЦИЯ И НАСТРОЙКИ СКОРОСТИ
// ==========================================
constexpr uint8_t RX_PIN = A0;                         // Аналоговый вход фотодиода
constexpr uint16_t BAUD_RATE = 30;                     // Скорость: 30 бит/с (33.3 мс)
constexpr uint32_t BIT_PERIOD_US = 1000000UL / BAUD_RATE; // 33 333 мкс

// Буфер для сборки UTF-8 текста любой длины
constexpr size_t BUFFER_SIZE = 512;
char sentenceBuffer[BUFFER_SIZE];
size_t bufferIndex = 0;

// Таймаут тишины (окончание всей фразы): 500 мс
constexpr uint32_t MESSAGE_TIMEOUT_MS = 500;
uint32_t lastCharTime = 0;
uint32_t messageStartTime = 0;
bool isReceivingMessage = false;

// Состояние приема пакета
enum RxPacketState {
    STATE_PAYLOAD,
    STATE_WAIT_CRC
};
RxPacketState rxState = STATE_PAYLOAD;

// Динамические параметры адаптивного порога и телеметрии
int ambientNoiseLevel = 3;                              // Уровень темноты
int thresholdValue = 35;                               // Автоматический адаптивный порог
int peakLightAdc = 150;                                // Замеренный пик света
int hysteresisVal = 8;                                 // Гистерезис

// Накопление средней яркости для точного расчета SNR
long totalLightAdcSum = 0;
int lightSamplesCount = 0;

// ==========================================
// ПРОТОТИПЫ ФУНКЦИЙ
// ==========================================
uint8_t calculateCRC8(const uint8_t* data, size_t len);
void calibrateDarkness();
bool checkStartTrigger();
bool sampleBitWithVoting(uint32_t bitCenterUs);
bool receiveByte(uint8_t& outByte);
void processIncomingByte(uint8_t byteVal);
void finalizeMessageWithCRC(bool hasCrc, uint8_t receivedCRC);
void handleSerialCommands();

// ==========================================
// SETUP
// ==========================================
void setup() {
    pinMode(RX_PIN, INPUT);

    Serial.begin(115200);
    delay(500);

    Serial.println(F("\n============================================================"));
    Serial.println(F("  >>> Li-Fi ПРИЕМНИК (RX) [ТЕЛЕМЕТРИЯ SNR + UTF-8 + CRC] <<<"));
    Serial.println(F("============================================================"));
    Serial.print(F("[INFO] Скорость Li-Fi: "));
    Serial.print(BAUD_RATE);
    Serial.print(F(" бод | Длительность бита: "));
    Serial.print(BIT_PERIOD_US / 1000);
    Serial.println(F(" мс"));
    Serial.println(F("[INFO] Включен расчет оптического контраста, SNR (дБ) и скорости."));

    // Замеряем базовый уровень темноты
    calibrateDarkness();

    Serial.println(F("------------------------------------------------------------"));
    Serial.println(F("Приемник готов к работе...\n"));
}

// ==========================================
// ГЛАВНЫЙ ЦИКЛ (LOOP)
// ==========================================
void loop() {
    handleSerialCommands();

    // Проверка появления оптического импульса (триггер старт-бита)
    if (checkStartTrigger()) {
        uint8_t receivedByte = 0;

        if (receiveByte(receivedByte)) {
            processIncomingByte(receivedByte);
        }
    }

    // Завершение фразы по таймауту
    if (isReceivingMessage && (millis() - lastCharTime > MESSAGE_TIMEOUT_MS)) {
        finalizeMessageWithCRC(false, 0);
    }
}

// ==========================================
// РАСЧЕТ CRC-8
// ==========================================

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

// ==========================================
// АВТОАДАПТИВНЫЙ ПРИЕМ
// ==========================================

bool checkStartTrigger() {
    int val = analogRead(RX_PIN);
    return (val > (ambientNoiseLevel + 12));
}

bool sampleBitWithVoting(uint32_t bitCenterUs) {
    int highVotes = 0;
    constexpr int32_t offsetsUs[3] = {-2000, 0, 2000};

    for (int i = 0; i < 3; i++) {
        uint32_t sampleTimeUs = bitCenterUs + offsetsUs[i];
        while ((long)(micros() - sampleTimeUs) < 0);
        
        int val = analogRead(RX_PIN);
        if (val >= thresholdValue) {
            highVotes++;
        }
    }

    return (highVotes >= 2);
}

/**
 * @brief Прием байта с автоматической динамической подстройкой порога
 */
bool receiveByte(uint8_t& outByte) {
    uint32_t frameStartUs = micros();

    // 1. Центр стартового бита (+ 0.5 * T = 16.6 мс)
    uint32_t startCenterUs = frameStartUs + (BIT_PERIOD_US / 2);
    while ((long)(micros() - startCenterUs) < 0);

    int startSample = analogRead(RX_PIN);
    if (startSample <= (ambientNoiseLevel + 15)) {
        return false; // Ложный шум
    }

    // ДИНАМИЧЕСКИЙ РАСЧЕТ ПОРОГА
    peakLightAdc = startSample;
    thresholdValue = (ambientNoiseLevel + peakLightAdc) / 2;
    hysteresisVal = max(4, (peakLightAdc - ambientNoiseLevel) / 10);

    // Сбор статистики для SNR
    totalLightAdcSum += peakLightAdc;
    lightSamplesCount++;

    uint8_t reconstructedByte = 0;

    // 2. Считывание 8 бит
    for (uint8_t bitIdx = 0; bitIdx < 8; bitIdx++) {
        uint32_t bitCenterUs = frameStartUs + (BIT_PERIOD_US * 3 / 2) + ((uint32_t)bitIdx * BIT_PERIOD_US);

        bool bitVal = sampleBitWithVoting(bitCenterUs);
        if (bitVal) {
            reconstructedByte |= (1 << bitIdx);
        }
    }

    // 3. Проверка стоп-бита (+ 9.5 * T)
    uint32_t stopCenterUs = frameStartUs + (BIT_PERIOD_US * 19 / 2);
    while ((long)(micros() - stopCenterUs) < 0);

    int stopAdc = analogRead(RX_PIN);
    bool isStopValid = (stopAdc < thresholdValue);

    // Окончание кадра (+ 10.0 * T)
    uint32_t frameEndUs = frameStartUs + (BIT_PERIOD_US * 10);
    while ((long)(micros() - frameEndUs) < 0);

    if (isStopValid) {
        outByte = reconstructedByte;
        return true;
    }

    return false;
}

// ==========================================
// ОБРАБОТКА СИМВОЛОВ И UTF-8
// ==========================================

void processIncomingByte(uint8_t byteVal) {
    if (!isReceivingMessage) {
        isReceivingMessage = true;
        messageStartTime = millis();
        totalLightAdcSum = 0;
        lightSamplesCount = 0;
    }
    lastCharTime = millis();

    // Принят контрольный байт CRC-8
    if (rxState == STATE_WAIT_CRC) {
        finalizeMessageWithCRC(true, byteVal);
        return;
    }

    // Маркер окончания текста
    if (byteVal == '\n' || byteVal == '\r') {
        rxState = STATE_WAIT_CRC;
        return;
    }

    // Вывод символа (ASCII + UTF-8 русские буквы)
    if (byteVal >= 32 && byteVal != 127) {
        Serial.write(byteVal);

        if (bufferIndex < BUFFER_SIZE - 1) {
            sentenceBuffer[bufferIndex++] = static_cast<char>(byteVal);
            sentenceBuffer[bufferIndex] = '\0';
        }
    }
}

/**
 * @brief Итоговый отчет: Сообщение + CRC-8 + Полная метрика качества канала Li-Fi
 */
void finalizeMessageWithCRC(bool hasCrc, uint8_t receivedCRC) {
    isReceivingMessage = false;
    rxState = STATE_PAYLOAD;

    if (bufferIndex == 0) return;

    // Расчет времени передачи
    uint32_t totalDurationMs = lastCharTime - messageStartTime + 350;
    if (totalDurationMs < 100) totalDurationMs = 100;
    float durationSec = totalDurationMs / 1000.0;

    // Расчет скорости
    float bytesPerSec = (float)bufferIndex / durationSec;
    float bitsPerSec = bytesPerSec * 8.0;

    // Расчет средней яркости луча и оптического контраста
    int avgLightAdc = (lightSamplesCount > 0) ? (int)(totalLightAdcSum / lightSamplesCount) : peakLightAdc;
    int contrastDelta = avgLightAdc - ambientNoiseLevel;

    // Расчет отношения Сигнал/Шум (SNR в дБ)
    float snrDb = 0.0;
    int noiseBase = max(1, ambientNoiseLevel);
    if (avgLightAdc > noiseBase) {
        snrDb = 20.0 * log10((float)avgLightAdc / (float)noiseBase);
    }

    // Расчет CRC-8
    uint8_t calculatedCRC = calculateCRC8(reinterpret_cast<const uint8_t*>(sentenceBuffer), bufferIndex);

    Serial.println();
    Serial.println(F("\n************************************************************"));
    Serial.print(F(">>> [ИТОГОВОЕ СООБЩЕНИЕ]: \""));
    Serial.print(sentenceBuffer);
    Serial.println(F("\""));
    Serial.print(F(">>> [РАЗМЕР СООБЩЕНИЯ]:  "));
    Serial.print(bufferIndex);
    Serial.println(F(" байт"));

    if (hasCrc) {
        Serial.print(F(">>> [КОНТРОЛЬ CRC-8]:     Расчетный = 0x"));
        if (calculatedCRC < 16) Serial.print(F("0"));
        Serial.print(calculatedCRC, HEX);
        Serial.print(F(" | Принятый = 0x"));
        if (receivedCRC < 16) Serial.print(F("0"));
        Serial.println(receivedCRC, HEX);

        if (calculatedCRC == receivedCRC) {
            Serial.println(F(">>> [СТАТУС ЦЕЛОСТНОСТИ]: [УСПЕШНО - ОШИБОК НЕТ!] ✔"));
        } else {
            Serial.println(F(">>> [СТАТУС ЦЕЛОСТНОСТИ]: [ОШИБКА CRC! ДАННЫЕ ИСКАЖЕНЫ] ❌"));
        }
    } else {
        Serial.print(F(">>> [КОНТРОЛЬ CRC-8]:     Таймаут CRC (Расчетный: 0x"));
        if (calculatedCRC < 16) Serial.print(F("0"));
        Serial.print(calculatedCRC, HEX);
        Serial.println(F(")"));
    }

    // ВЫВОД ТЕЛЕМЕТРИИ КАНАЛА
    Serial.println(F("------------------------------------------------------------"));
    Serial.println(F(">>> [МЕТРИКИ ОПТИЧЕСКОГО КАНАЛА LI-FI]:"));
    Serial.print(F("    • Оптический контраст (ΔV): "));
    Serial.print(contrastDelta);
    Serial.print(F(" ADC (Луч: "));
    Serial.print(avgLightAdc);
    Serial.print(F(" | Фон: "));
    Serial.print(ambientNoiseLevel);
    Serial.println(F(")"));

    Serial.print(F("    • SNR (Отношение Сигнал/Шум): "));
    Serial.print(snrDb, 1);
    Serial.print(F(" dB "));
    if (snrDb >= 25.0) {
        Serial.println(F("[ОТЛИЧНЫЙ СИГНАЛ] ★★★"));
    } else if (snrDb >= 16.0) {
        Serial.println(F("[ХОРОШИЙ СИГНАЛ] ★★☆"));
    } else if (snrDb >= 8.0) {
        Serial.println(F("[УДОВЛЕТВОРИТЕЛЬНО] ★☆☆"));
    } else {
        Serial.println(F("[СЛАБЫЙ СИГНАЛ / ШУМ] ☆☆☆"));
    }

    Serial.print(F("    • Полезная скорость передачи: "));
    Serial.print(bytesPerSec, 1);
    Serial.print(F(" байт/с ("));
    Serial.print(bitsPerSec, 1);
    Serial.println(F(" бит/с)"));

    Serial.print(F("    • Время передачи сообщения:   "));
    Serial.print(durationSec, 1);
    Serial.println(F(" сек"));
    Serial.println(F("************************************************************\n"));

    bufferIndex = 0;
    sentenceBuffer[0] = '\0';
}

// ==========================================
// КАЛИБРОВКА ТЕМНОТЫ И КОМАНДЫ
// ==========================================

void calibrateDarkness() {
    long sum = 0;
    constexpr int SAMPLES = 60;
    int maxVal = 0;

    for (int i = 0; i < SAMPLES; i++) {
        int val = analogRead(RX_PIN);
        sum += val;
        if (val > maxVal) maxVal = val;
        delay(10);
    }

    ambientNoiseLevel = sum / SAMPLES;
    thresholdValue = ambientNoiseLevel + 25;

    Serial.print(F("[КАЛИБРОВКА] Фоновая темнота: "));
    Serial.print(ambientNoiseLevel);
    Serial.println(F(" (ADC 0..1023)\n"));
}

void handleSerialCommands() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'c' || cmd == 'C') {
            calibrateDarkness();
        } else if (cmd == 'r' || cmd == 'R') {
            int cur = analogRead(RX_PIN);
            Serial.print(F("[АЦП]: "));
            Serial.print(cur);
            Serial.print(F(" | Фон: "));
            Serial.print(ambientNoiseLevel);
            Serial.print(F(" | Порог: "));
            Serial.println(thresholdValue);
        }
    }
}
