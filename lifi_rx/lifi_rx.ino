/*
 * ============================================================================
 * ПРОЕКТ: Li-Fi Односторонняя оптическая связь (Visible Light Communication)
 * МОДУЛЬ: ПРИЕМНИК (RX) - СТАБИЛЬНЫЙ РЕЖИМ 30 БОД + CRC-8 + 3X ОВЕРСЭМПЛИНГ
 * ПЛАТФОРМА: Arduino Uno (ATmega328P)
 * ДАТЧИК: Фотодиод BPW24 (Катод -> 5V, Анод -> A0, Резистор 10 кОм -> GND)
 * ============================================================================
 */

#include <Arduino.h>

// ==========================================
// КОНФИГУРАЦИЯ И НАСТРОЙКИ СКОРОСТИ
// ==========================================
constexpr uint8_t RX_PIN = A0;                         // Аналоговый вход фотодиода
constexpr uint16_t BAUD_RATE = 30;                     // Оптимальная скорость: 30 бит/с (33.3 мс)
constexpr uint32_t BIT_PERIOD_US = 1000000UL / BAUD_RATE; // 33 333 мкс

// Буфер для сборки предложения
constexpr size_t BUFFER_SIZE = 256;
char sentenceBuffer[BUFFER_SIZE];
size_t bufferIndex = 0;

// Таймаут тишины (окончание всей фразы): 500 мс
constexpr uint32_t MESSAGE_TIMEOUT_MS = 500;
uint32_t lastCharTime = 0;
bool isReceivingMessage = false;

// Состояние приема пакета
enum RxPacketState {
    STATE_PAYLOAD,      // Прием полезного текста
    STATE_WAIT_CRC      // Ожидание контрольного байта CRC-8
};
RxPacketState rxState = STATE_PAYLOAD;

// Порог срабатывания и параметры калибровки
int ambientNoiseLevel = 0;                              // Базовый уровень шума (темнота)
int thresholdValue = 60;                               // Порог переключения (ADC 0..1023)
constexpr int NOISE_MARGIN = 40;                        // Запас над шумом
constexpr int HYSTERESIS = 12;                          // Гистерезис

// ==========================================
// ПРОТОТИПЫ ФУНКЦИЙ
// ==========================================
uint8_t calculateCRC8(const uint8_t* data, size_t len);
void calibrateAmbientLight();
bool isLightPresent();
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

    // Запуск Serial Monitor на 115200 бод
    Serial.begin(115200);
    delay(500);

    Serial.println(F("\n============================================================"));
    Serial.println(F("     >>> Li-Fi ПРИЕМНИК (RX) [30 BAUD + CRC-8] <<<          "));
    Serial.println(F("============================================================"));
    Serial.print(F("[INFO] Скорость Li-Fi: "));
    Serial.print(BAUD_RATE);
    Serial.print(F(" бод | Длительность бита: "));
    Serial.print(BIT_PERIOD_US / 1000);
    Serial.println(F(" мс (Оптимальная стабильность)"));
    Serial.println(F("[INFO] В темноте - тишина (спам отключен)."));

    // Автокалибровка порога под освещение
    calibrateAmbientLight();

    Serial.println(F("------------------------------------------------------------"));
    Serial.println(F("Команды: 'c' - перекалибровка, '+' / '-' - подстройка порога, 'r' - замер"));
    Serial.println(F("Ожидание оптического сигнала...\n"));
}

// ==========================================
// ГЛАВНЫЙ ЦИКЛ (LOOP)
// ==========================================
void loop() {
    // 1. Проверка команд управления от пользователя
    handleSerialCommands();

    // 2. Ожидание оптического старт-бита
    if (isLightPresent()) {
        uint8_t receivedByte = 0;

        // Прием байта
        if (receiveByte(receivedByte)) {
            processIncomingByte(receivedByte);
        }
    }

    // 3. Завершение фразы по таймауту тишины
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
// АЛГОРИТМ ПРИЕМА И ВЫБОРКА
// ==========================================

bool isLightPresent() {
    return (analogRead(RX_PIN) >= thresholdValue);
}

/**
 * @brief 3-кратное сэмплирование в центре бита (-2мс, 0мс, +2мс)
 * Идеально для 33.3 мс бита: фильтрует помехи и не задевает спад фотодиода
 */
bool sampleBitWithVoting(uint32_t bitCenterUs) {
    int highVotes = 0;
    constexpr int32_t offsetsUs[3] = {-2000, 0, 2000};

    for (int i = 0; i < 3; i++) {
        uint32_t sampleTimeUs = bitCenterUs + offsetsUs[i];
        while ((long)(micros() - sampleTimeUs) < 0);
        
        int val = analogRead(RX_PIN);
        if (val >= (thresholdValue - HYSTERESIS)) {
            highVotes++;
        }
    }

    return (highVotes >= 2);
}

/**
 * @brief Прецизионный прием байта с абсолютной микросекундной фазировкой
 */
bool receiveByte(uint8_t& outByte) {
    uint32_t frameStartUs = micros();

    // Шаг 1: Центр стартового бита (+ 0.5 * T)
    uint32_t startCenterUs = frameStartUs + (BIT_PERIOD_US / 2);
    while ((long)(micros() - startCenterUs) < 0);

    int startSample = analogRead(RX_PIN);
    if (startSample < (thresholdValue - HYSTERESIS)) {
        return false; // Ложный блик
    }

    uint8_t reconstructedByte = 0;

    // Шаг 2: Считывание 8 бит
    for (uint8_t bitIdx = 0; bitIdx < 8; bitIdx++) {
        uint32_t bitCenterUs = frameStartUs + (BIT_PERIOD_US * 3 / 2) + ((uint32_t)bitIdx * BIT_PERIOD_US);

        bool bitVal = sampleBitWithVoting(bitCenterUs);

        if (bitVal) {
            reconstructedByte |= (1 << bitIdx);
        }
    }

    // Шаг 3: Проверка стоп-бита (+ 9.5 * T)
    uint32_t stopCenterUs = frameStartUs + (BIT_PERIOD_US * 19 / 2);
    while ((long)(micros() - stopCenterUs) < 0);

    int stopAdc = analogRead(RX_PIN);
    bool isStopValid = (stopAdc < thresholdValue);

    // Окончание стоп-бита (+ 10.0 * T)
    uint32_t frameEndUs = frameStartUs + (BIT_PERIOD_US * 10);
    while ((long)(micros() - frameEndUs) < 0);

    if (isStopValid) {
        outByte = reconstructedByte;
        return true;
    }

    return false;
}

// ==========================================
// ОБРАБОТКА СИМВОЛОВ И ПРОВЕРКА CRC-8
// ==========================================

void processIncomingByte(uint8_t byteVal) {
    lastCharTime = millis();
    isReceivingMessage = true;
    char ch = static_cast<char>(byteVal);

    // Если ждем контрольный байт CRC-8
    if (rxState == STATE_WAIT_CRC) {
        finalizeMessageWithCRC(true, byteVal);
        return;
    }

    // Если пришел маркер окончания текста -> переключаемся в ожидание CRC-8
    if (ch == '\n' || ch == '\r') {
        rxState = STATE_WAIT_CRC;
        return;
    }

    // Быстрый вывод символа в поток
    if (ch >= 32 && ch <= 126) {
        Serial.print(ch);

        if (bufferIndex < BUFFER_SIZE - 1) {
            sentenceBuffer[bufferIndex++] = ch;
            sentenceBuffer[bufferIndex] = '\0';
        }
    }
}

/**
 * @brief Итоговая проверка контрольной суммы CRC-8 и вывод отчета
 */
void finalizeMessageWithCRC(bool hasCrc, uint8_t receivedCRC) {
    isReceivingMessage = false;
    rxState = STATE_PAYLOAD;

    if (bufferIndex == 0) return;

    // Рассчитываем контрольную сумму от полученных данных
    uint8_t calculatedCRC = calculateCRC8(reinterpret_cast<const uint8_t*>(sentenceBuffer), bufferIndex);

    Serial.println();
    Serial.println(F("\n************************************************************"));
    Serial.print(F(">>> [ИТОГОВОЕ СООБЩЕНИЕ]: \""));
    Serial.print(sentenceBuffer);
    Serial.println(F("\""));
    Serial.print(F(">>> [ДЛИНА СООБЩЕНИЯ]:   "));
    Serial.print(bufferIndex);
    Serial.println(F(" символов"));

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
            Serial.println(F(">>> [СТАТУС ЦЕЛОСТНОСТИ]: [ОШИБКА CRC! ДАННЫЕ ИСКАЖЕНЫ ПОМЕХОЙ] ❌"));
        }
    } else {
        Serial.print(F(">>> [КОНТРОЛЬ CRC-8]:     Таймаут ожидания CRC (Расчетный: 0x"));
        if (calculatedCRC < 16) Serial.print(F("0"));
        Serial.print(calculatedCRC, HEX);
        Serial.println(F(")"));
    }
    Serial.println(F("************************************************************\n"));

    bufferIndex = 0;
    sentenceBuffer[0] = '\0';
}

// ==========================================
// КАЛИБРОВКА И УПРАВЛЕНИЕ
// ==========================================

void calibrateAmbientLight() {
    Serial.println(F("\n[КАЛИБРОВКА] Измерение фонового света..."));
    Serial.println(F("[КАЛИБРОВКА] Убедитесь, что передающий LED выключен."));

    long sum = 0;
    constexpr int SAMPLES = 80;
    int minVal = 1023;
    int maxVal = 0;

    for (int i = 0; i < SAMPLES; i++) {
        int val = analogRead(RX_PIN);
        sum += val;
        if (val < minVal) minVal = val;
        if (val > maxVal) maxVal = val;
        delay(12);
    }

    ambientNoiseLevel = sum / SAMPLES;
    thresholdValue = max(ambientNoiseLevel + NOISE_MARGIN, maxVal + 15);
    thresholdValue = constrain(thresholdValue, 25, 950);

    Serial.print(F("[КАЛИБРОВКА] Фон: "));
    Serial.print(ambientNoiseLevel);
    Serial.print(F(", Пик: "));
    Serial.print(maxVal);
    Serial.print(F(" -> Порог (Threshold): "));
    Serial.print(thresholdValue);
    Serial.println(F(" (ADC 0..1023)\n"));
}

void handleSerialCommands() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'c' || cmd == 'C') {
            calibrateAmbientLight();
        } else if (cmd == '+') {
            thresholdValue = min(1000, thresholdValue + 15);
            Serial.print(F("[НАСТРОЙКА] Порог увеличен: "));
            Serial.println(thresholdValue);
        } else if (cmd == '-') {
            thresholdValue = max(10, thresholdValue - 15);
            Serial.print(F("[НАСТРОЙКА] Порог уменьшен: "));
            Serial.println(thresholdValue);
        } else if (cmd == 'r' || cmd == 'R') {
            int cur = analogRead(RX_PIN);
            Serial.print(F("[ЗАМЕР АЦП]: "));
            Serial.print(cur);
            Serial.print(F(" / 1023 | Порог: "));
            Serial.print(thresholdValue);
            Serial.println((cur >= thresholdValue) ? F(" [СВЕТ ОБНАРУЖЕН]") : F(" [ТЕМНОТА]"));
        }
    }
}
