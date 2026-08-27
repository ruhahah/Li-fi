/*
 * ============================================================================
 * ПРОЕКТ: Li-Fi Односторонняя оптическая связь (Visible Light Communication)
 * МОДУЛЬ: ПРИЕМНИК (RX) - ПОДДЕРЖКА ПРОБЕЛОВ И МНОГОСЛОВНЫХ ПРЕДЛОЖЕНИЙ
 * ПЛАТФОРМА: Arduino Uno (ATmega328P)
 * ДАТЧИК: Фотодиод BPW24 (Катод -> 5V, Анод -> A0, Резистор 10 кОм -> GND)
 * ============================================================================
 */

#include <Arduino.h>

// ==========================================
// КОНФИГУРАЦИЯ И НАСТРОЙКИ
// ==========================================
constexpr uint8_t RX_PIN = A0;                        // Аналоговый вход фотодиода
constexpr uint16_t BAUD_RATE = 20;                    // Скорость передачи: 20 бит/с
constexpr uint32_t BIT_PERIOD_MS = 1000 / BAUD_RATE;  // Длительность одного бита: 50 мс

// Буфер для сборки предложений и фраз с пробелами
constexpr size_t BUFFER_SIZE = 256;
char messageBuffer[BUFFER_SIZE];
size_t bufferIndex = 0;

// Таймаут тишины (окончание всей передачи / полная остановка)
constexpr uint32_t MESSAGE_FLUSH_TIMEOUT_MS = 600;    // 600 мс тишины = фраза завершена
uint32_t lastCharTime = 0;
bool isReceivingMessage = false;

// Параметры порога и калибровки
int ambientNoiseLevel = 0;                             // Базовый фоновый шум
int thresholdValue = 200;                              // Порог переключения (ADC 0..1023)
constexpr int NOISE_MARGIN = 50;                       // Запас над шумом
constexpr int HYSTERESIS = 15;                         // Гистерезис

// Таймер статусного отчета (Heartbeat) - 1 раз в 3 секунды
uint32_t lastTelemetryTime = 0;
constexpr uint32_t TELEMETRY_INTERVAL_MS = 3000;       // 3 секунды
bool liveStreamEnabled = true;

// ==========================================
// ПРОТОТИПЫ ФУНКЦИЙ
// ==========================================
void calibrateAmbientLight();
bool isLightPresent();
bool receiveByte(uint8_t& outByte);
void processIncomingByte(char ch);
void flushMessageBuffer();
void handleSerialCommands();
void printTelemetryReport();

// ==========================================
// SETUP
// ==========================================
void setup() {
    pinMode(RX_PIN, INPUT);

    // Скорость Serial 115200 бод для мгновенного вывода без задержек
    Serial.begin(115200);
    delay(500);

    Serial.println(F("\n======================================================="));
    Serial.println(F("    >>> Li-Fi ПРИЕМНИК (RX) - ПРИЕМ ФРАЗ И ПРЕДЛОЖЕНИЙ <<<  "));
    Serial.println(F("======================================================="));
    Serial.print(F("[INFO] Скорость Li-Fi: "));
    Serial.print(BAUD_RATE);
    Serial.print(F(" бод | Длительность бита: "));
    Serial.print(BIT_PERIOD_MS);
    Serial.println(F(" мс"));
    Serial.println(F("[INFO] Пробелы сохраняются внутри фразы."));
    Serial.println(F("[INFO] Завершение фразы: символ '\\n' или пауза 600 мс."));

    // Калибровка под освещение
    calibrateAmbientLight();

    Serial.println(F("-------------------------------------------------------"));
    Serial.println(F("Команды: 'c' - калибровка, '+' / '-' - порог, 'm' - статус 3с"));
    Serial.println(F("Ожидание оптического сигнала...\n"));
}

// ==========================================
// ГЛАВНЫЙ ЦИКЛ (LOOP)
// ==========================================
void loop() {
    // 1. Проверка команд управления
    handleSerialCommands();

    // 2. Ожидание оптического старт-бита
    if (isLightPresent()) {
        uint8_t receivedByte = 0;

        if (receiveByte(receivedByte)) {
            char ch = static_cast<char>(receivedByte);
            processIncomingByte(ch);
        }
    }

    // 3. Завершение фразы по таймауту тишины (полная остановка)
    if (isReceivingMessage && (millis() - lastCharTime > MESSAGE_FLUSH_TIMEOUT_MS)) {
        flushMessageBuffer();
    }

    // 4. Отчет каждые 3 секунды (только если сейчас не идет прием фразы)
    if (!isReceivingMessage && liveStreamEnabled && (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL_MS)) {
        lastTelemetryTime = millis();
        printTelemetryReport();
    }
}

// ==========================================
// ФУНКЦИИ СИНХРОНИЗАЦИИ И ПРИЕМА
// ==========================================

bool isLightPresent() {
    return (analogRead(RX_PIN) >= thresholdValue);
}

/**
 * @brief Прецизионный прием одного байта по протоколу UART (8-N-1)
 */
bool receiveByte(uint8_t& outByte) {
    // Шаг 1: Центр стартового бита (T/2 = 25 мс)
    delay(BIT_PERIOD_MS / 2);
    int sample = analogRead(RX_PIN);
    if (sample < (thresholdValue - HYSTERESIS)) {
        return false; // Ложный импульс/шум
    }

    // Шаг 2: Центр Бита 0 (+50 мс)
    delay(BIT_PERIOD_MS);

    uint8_t reconstructedByte = 0;

    // Шаг 3: Считывание 8 бит данных строго в центрах
    for (uint8_t bitIdx = 0; bitIdx < 8; bitIdx++) {
        uint32_t bitStart = millis();

        int bitAdc = analogRead(RX_PIN);
        if (bitAdc >= (thresholdValue - HYSTERESIS)) {
            reconstructedByte |= (1 << bitIdx);
        }

        // Прецизионное ожидание 50 мс
        while (millis() - bitStart < BIT_PERIOD_MS) {
            // NOP
        }
    }

    // Шаг 4: Проверка стопового бита (должен быть LOW)
    int stopAdc = analogRead(RX_PIN);
    bool isStopValid = (stopAdc < thresholdValue);

    // Дожидаемся окончания стоп-бита (20 мс)
    delay(BIT_PERIOD_MS / 2);

    if (isStopValid) {
        outByte = reconstructedByte;
        return true;
    }

    return false;
}

// ==========================================
// ОБРАБОТКА СИМВОЛОВ И БУФЕРА
// ==========================================

void processIncomingByte(char ch) {
    lastCharTime = millis();
    isReceivingMessage = true;

    // 1. Быстрый визуальный вывод принятого символа в поток
    if (ch == ' ') {
        Serial.print(F("[ПРОБЕЛ] "));
    } else if (ch == '\n' || ch == '\r') {
        Serial.println(F("[КОНЕЦ СТРОКИ]"));
    } else if (ch >= 32 && ch <= 126) {
        Serial.print(F("'"));
        Serial.print(ch);
        Serial.print(F("' "));
    }

    // 2. Если пришел символ конца строки (\n) -> сразу завершаем фразу
    if (ch == '\n' || ch == '\r') {
        flushMessageBuffer();
        return;
    }

    // 3. Сохраняем символ (включая ПРОБЕЛЫ) в общий буфер сообщения
    if (ch >= 32 && ch <= 126) {
        if (bufferIndex < BUFFER_SIZE - 1) {
            messageBuffer[bufferIndex++] = ch;
            messageBuffer[bufferIndex] = '\0';
        } else {
            // Переполнение буфера
            flushMessageBuffer();
            messageBuffer[bufferIndex++] = ch;
            messageBuffer[bufferIndex] = '\0';
        }
    }
}

/**
 * @brief Вывод всей принятой фразы (с пробелами и несколькими словами)
 */
void flushMessageBuffer() {
    isReceivingMessage = false;

    if (bufferIndex == 0) return;

    Serial.println();
    Serial.println(F("\n======================================================="));
    Serial.print(F(">>> [ИТОГОВОЕ СООБЩЕНИЕ]: \""));
    Serial.print(messageBuffer);
    Serial.println(F("\""));
    Serial.print(F(">>> [ДЛИНА]: "));
    Serial.print(bufferIndex);
    Serial.println(F(" символов"));
    Serial.println(F("=======================================================\n"));

    bufferIndex = 0;
    messageBuffer[0] = '\0';
}

// ==========================================
// ПЕРИОДИЧЕСКИЙ ОТЧЕТ (РАЗ В 3 СЕКУНДЫ)
// ==========================================

void printTelemetryReport() {
    int curAdc = analogRead(RX_PIN);
    bool lightState = (curAdc >= thresholdValue);

    Serial.print(F("[СТАТУС 3 сек] Датчик А0: "));
    if (curAdc < 100) Serial.print(F(" "));
    if (curAdc < 10)  Serial.print(F(" "));
    Serial.print(curAdc);
    Serial.print(F(" / 1023 | Порог: "));
    Serial.print(thresholdValue);
    Serial.print(F(" | Шум: "));
    Serial.print(ambientNoiseLevel);
    
    if (lightState) {
        Serial.println(F(" | Состояние: [СВЕТ ОБНАРУЖЕН]"));
    } else {
        Serial.println(F(" | Состояние: [ОЖИДАНИЕ СИГНАЛА]"));
    }
}

// ==========================================
// КАЛИБРОВКА И УПРАВЛЕНИЕ
// ==========================================

void calibrateAmbientLight() {
    Serial.println(F("\n[КАЛИБРОВКА] Анализ освещения в помещении (1.5 сек)..."));

    long sum = 0;
    constexpr int SAMPLES = 100;
    int minVal = 1023;
    int maxVal = 0;

    for (int i = 0; i < SAMPLES; i++) {
        int val = analogRead(RX_PIN);
        sum += val;
        if (val < minVal) minVal = val;
        if (val > maxVal) maxVal = val;
        delay(15);
    }

    ambientNoiseLevel = sum / SAMPLES;
    thresholdValue = max(ambientNoiseLevel + NOISE_MARGIN, maxVal + (NOISE_MARGIN / 2));
    thresholdValue = constrain(thresholdValue, 30, 950);

    Serial.print(F("[КАЛИБРОВКА] Фоновый шум: "));
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
            thresholdValue = min(1000, thresholdValue + 20);
            Serial.print(F("[НАСТРОЙКА] Порог увеличен: "));
            Serial.println(thresholdValue);
        } else if (cmd == '-') {
            thresholdValue = max(10, thresholdValue - 20);
            Serial.print(F("[НАСТРОЙКА] Порог уменьшен: "));
            Serial.println(thresholdValue);
        } else if (cmd == 'm' || cmd == 'M') {
            liveStreamEnabled = !liveStreamEnabled;
            Serial.print(F("[НАСТРОЙКА] Отчет 3с: "));
            Serial.println(liveStreamEnabled ? F("ВКЛ") : F("ВЫКЛ"));
        }
    }
}
