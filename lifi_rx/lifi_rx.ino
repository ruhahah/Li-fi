/*
 * ============================================================================
 * ПРОЕКТ: Li-Fi Односторонняя оптическая связь (Visible Light Communication)
 * МОДУЛЬ: ПРИЕМНИК (RX)
 * ПЛАТФОРМА: Arduino Uno (ATmega328P)
 * ДАТЧИК: Фотодиод PHYWE BPW24 + Pull-down 10 кОм на Pin A0
 * ============================================================================
 */

#include <Arduino.h>

// ==========================================
// КОНФИГУРАЦИЯ И НАСТРОЙКИ
// ==========================================
constexpr uint8_t RX_PIN = A0;               // Аналоговый вход фотодиода
constexpr uint16_t BAUD_RATE = 20;           // Скорость передачи (должна совпадать с TX): 20 бит/с
constexpr uint32_t BIT_PERIOD_MS = 1000 / BAUD_RATE; // Длительность бита: 50 мс

// Буфер для сборки текста в слова и предложения
constexpr size_t BUFFER_SIZE = 128;
char wordBuffer[BUFFER_SIZE];
size_t bufferIndex = 0;

// Таймаут завершения слова (если слово оборвалось без пробела)
constexpr uint32_t WORD_FLUSH_TIMEOUT_MS = 800;
uint32_t lastCharTime = 0;

// Параметры калибровки и порогового компаратора
int ambientNoiseLevel = 0;                    // Базовый уровень фонового света
int thresholdValue = 250;                     // Порог переключения (ADC 0..1023)
constexpr int NOISE_MARGIN = 60;              // Запас над фоновым шумом для автоматического порога
constexpr int HYSTERESIS = 15;                // Гистерезис для подавления оптического шума

// ==========================================
// ПРОТОТИПЫ ФУНКЦИЙ
// ==========================================
void calibrateAmbientLight();
bool isLightPresent();
bool readBitSample();
bool receiveByte(uint8_t& outByte);
void processIncomingByte(char ch);
void flushWordBuffer();
void handleSerialCommands();

// ==========================================
// SETUP
// ==========================================
void setup() {
    pinMode(RX_PIN, INPUT);

    // Высокая скорость Serial для монитора порта (не тормозит основной цикл)
    Serial.begin(115200);
    while (!Serial && millis() < 2000);

    Serial.println(F("\n========================================"));
    Serial.println(F("       Li-Fi RECEIVER (RX) START        "));
    Serial.println(F("========================================"));
    Serial.print(F("Speed: "));
    Serial.print(BAUD_RATE);
    Serial.print(F(" baud | Bit duration: "));
    Serial.print(BIT_PERIOD_MS);
    Serial.println(F(" ms"));

    // Автоматическая калибровка фонового освещения в помещении
    calibrateAmbientLight();

    Serial.println(F("Commands: Type 'c' to recalibrate, '+' / '-' to adjust threshold."));
    Serial.println(F("Listening for optical signal...\n"));
}

// ==========================================
// ГЛАВНЫЙ ЦИКЛ (LOOP)
// ==========================================
void loop() {
    // 1. Проверка команд пользователя из Serial Monitor
    handleSerialCommands();

    // 2. Ожидание оптического стартового бита (переход LOW -> HIGH)
    if (isLightPresent()) {
        uint8_t receivedByte = 0;
        
        // Попытка приема байта по протоколу
        if (receiveByte(receivedByte)) {
            char ch = static_cast<char>(receivedByte);
            processIncomingByte(ch);
        }
    }

    // 3. Сброс буфера по таймауту (если пауза между символами превысила лимит)
    if (bufferIndex > 0 && (millis() - lastCharTime > WORD_FLUSH_TIMEOUT_MS)) {
        flushWordBuffer();
    }
}

// ==========================================
// АЛГОРИТМ ПРИЕМА И ДЕКОДИРОВАНИЯ БАЙТА
// ==========================================

/**
 * @brief Проверка наличия света с гистерезисом.
 * @return true, если уровень сигнала выше порога (свет включен).
 */
bool isLightPresent() {
    int rawAdc = analogRead(RX_PIN);
    return rawAdc >= thresholdValue;
}

/**
 * @brief Чтение одного бита путем выборки в аналоговом пине.
 */
bool readBitSample() {
    int rawAdc = analogRead(RX_PIN);
    return rawAdc >= (thresholdValue - HYSTERESIS);
}

/**
 * @brief Прием одного байта с временной синхронизацией по центру бита.
 * 
 * Алгоритм синхронизации (Software UART Sampling):
 * 1. Обнаружен передний фронт (Start Bit Edge).
 * 2. Задержка T / 2 (25 мс при 20 бод): попадаем в центр Start-бита и проверяем валидность.
 * 3. Задержка T (50 мс): попадаем ровно в центр 1-го бита данных (Bit 0).
 * 4. Последовательно с шагом T опрашиваем биты 0..7 в их геометрических центрах.
 * 5. Задержка T: считываем Stop-бит (должен быть LOW / темнота).
 * 
 * @param outByte Ссылка для записи принятого байта.
 * @return true при успешном приеме и валидном стоп-бите.
 */
bool receiveByte(uint8_t& outByte) {
    uint32_t frameStartTime = millis();

    // Шаг 1: Ждем T/2, чтобы попасть в центр стартового бита (верификация от помех)
    delay(BIT_PERIOD_MS / 2);
    if (!readBitSample()) {
        // Ложное срабатывание (кратковременный блик/шум)
        return false;
    }

    // Шаг 2: Переходим к середине первого информационного бита (сдвиг на +1.0 * T)
    delay(BIT_PERIOD_MS);

    uint8_t reconstructedByte = 0;

    // Шаг 3: Считываем 8 бит данных (LSB first) с интервалом в 1 период
    for (uint8_t bitIdx = 0; bitIdx < 8; bitIdx++) {
        uint32_t sampleStart = millis();

        // Считываем значение бита в его центре
        if (readBitSample()) {
            reconstructedByte |= (1 << bitIdx);
        }

        // Прецизионное ожидание следующего битового интервала
        while (millis() - sampleStart < BIT_PERIOD_MS) {
            // NOP
        }
    }

    // Шаг 4: Проверка стопового бита (должен быть LOW)
    // Ждем половину периода стопового бита для сэмплирования
    delay(BIT_PERIOD_MS / 4);
    bool stopBitValid = !readBitSample(); // Стоповый бит корректен, если свет выключен

    if (stopBitValid) {
        outByte = reconstructedByte;
        return true;
    }

    // Ошибка кадрирования (Framing Error)
    return false;
}

// ==========================================
// ОБРАБОТКА И БУФЕРИЗАЦИЯ ТЕКСТА
// ==========================================

/**
 * @brief Сборка принятых символов в слова и предложения.
 * @param ch Принятый ASCII символ.
 */
void processIncomingByte(char ch) {
    lastCharTime = millis();

    // Фильтрация непечатных мусорных символов
    if (ch < 32 && ch != '\n' && ch != '\r' && ch != '\t') {
        return;
    }

    // Если символ — пробел или перенос строки, выводим накопленное слово
    if (ch == ' ' || ch == '\n' || ch == '\r') {
        if (bufferIndex > 0) {
            flushWordBuffer();
        }
        if (ch == '\n') {
            Serial.println(); // Дублируем перевод строки
        }
    } else {
        // Добавляем символ в буфер слова
        if (bufferIndex < BUFFER_SIZE - 1) {
            wordBuffer[bufferIndex++] = ch;
            wordBuffer[bufferIndex] = '\0';
        } else {
            // Переполнение буфера — принудительный сброс
            flushWordBuffer();
            wordBuffer[bufferIndex++] = ch;
            wordBuffer[bufferIndex] = '\0';
        }
    }
}

/**
 * @brief Вывод накопленного слова в Serial Monitor и очистка буфера.
 */
void flushWordBuffer() {
    if (bufferIndex == 0) return;

    Serial.print(F("[RX Word]: \""));
    Serial.print(wordBuffer);
    Serial.println(F("\""));

    bufferIndex = 0;
    wordBuffer[0] = '\0';
}

// ==========================================
// КАЛИБРОВКА И УПРАВЛЕНИЕ ПОРОГОМ
// ==========================================

/**
 * @brief Автоматическая калибровка уровня фонового света помещения.
 */
void calibrateAmbientLight() {
    Serial.println(F("[CALIBRATION] Measuring ambient light for 1.5 seconds..."));
    Serial.println(F("[CALIBRATION] Please ensure TX LED is OFF during calibration."));

    long sum = 0;
    constexpr int SAMPLES = 150;
    int minVal = 1023;
    int maxVal = 0;

    for (int i = 0; i < SAMPLES; i++) {
        int val = analogRead(RX_PIN);
        sum += val;
        if (val < minVal) minVal = val;
        if (val > maxVal) maxVal = val;
        delay(10);
    }

    ambientNoiseLevel = sum / SAMPLES;
    // Порог устанавливается выше пикового шума
    thresholdValue = max(ambientNoiseLevel + NOISE_MARGIN, maxVal + (NOISE_MARGIN / 2));

    Serial.print(F("[CALIBRATION] Ambient Noise: Avg = "));
    Serial.print(ambientNoiseLevel);
    Serial.print(F(", Peak = "));
    Serial.println(maxVal);
    Serial.print(F("[CALIBRATION] Optimal Threshold set to: "));
    Serial.print(thresholdValue);
    Serial.println(F(" (ADC 0..1023)\n"));
}

/**
 * @brief Интерактивное управление через Serial Monitor.
 */
void handleSerialCommands() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'c' || cmd == 'C') {
            calibrateAmbientLight();
        } else if (cmd == '+') {
            thresholdValue += 20;
            Serial.print(F("[INFO] Threshold increased to: "));
            Serial.println(thresholdValue);
        } else if (cmd == '-') {
            thresholdValue = max(0, thresholdValue - 20);
            Serial.print(F("[INFO] Threshold decreased to: "));
            Serial.println(thresholdValue);
        } else if (cmd == 'r' || cmd == 'R') {
            Serial.print(F("[RAW ADC]: "));
            Serial.print(analogRead(RX_PIN));
            Serial.print(F(" | Current Threshold: "));
            Serial.println(thresholdValue);
        }
    }
}
