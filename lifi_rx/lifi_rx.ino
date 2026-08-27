/*
 * ============================================================================
 * ПРОЕКТ: Li-Fi Односторонняя оптическая связь (Visible Light Communication)
 * МОДУЛЬ: ПРИЕМНИК (RX) - ВЕРСИЯ С ИСПРАВЛЕННОЙ СИНХРОНИЗАЦИЕЙ
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

// Буфер для сборки текста в слова и предложения
constexpr size_t BUFFER_SIZE = 128;
char wordBuffer[BUFFER_SIZE];
size_t bufferIndex = 0;

// Таймаут завершения слова (если слово закончилось без пробела)
constexpr uint32_t WORD_FLUSH_TIMEOUT_MS = 1200;
uint32_t lastCharTime = 0;

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
void flushWordBuffer();
void handleSerialCommands();
void printTelemetryReport();

// ==========================================
// SETUP
// ==========================================
void setup() {
    pinMode(RX_PIN, INPUT);

    // Высокая скорость Serial (115200 бод), чтобы вывод не тормозил прием
    Serial.begin(115200);
    delay(500); // Ожидание подключения терминала ПК

    Serial.println(F("\n======================================================="));
    Serial.println(F("          >>> Li-Fi ПРИЕМНИК (RX) ГОТОВ <<<            "));
    Serial.println(F("======================================================="));
    Serial.print(F("[INFO] Скорость: "));
    Serial.print(BAUD_RATE);
    Serial.print(F(" бод (бит/с) | Длительность бита: "));
    Serial.print(BIT_PERIOD_MS);
    Serial.println(F(" мс"));
    Serial.print(F("[INFO] Входной пин фотодиода: A"));
    Serial.println(RX_PIN - A0);
    Serial.println(F("[INFO] Отчет о работе датчика: каждые 3 секунды"));

    // Автокалибровка порога под освещение в комнате
    calibrateAmbientLight();

    Serial.println(F("-------------------------------------------------------"));
    Serial.println(F("Команды: 'c' - калибровка, '+' / '-' - порог, 'm' - вкл/выкл отчета 3с"));
    Serial.println(F("Ожидание оптического сигнала...\n"));
}

// ==========================================
// ГЛАВНЫЙ ЦИКЛ (LOOP)
// ==========================================
void loop() {
    // 1. Проверка команд от пользователя в Serial Monitor
    handleSerialCommands();

    // 2. Ожидание появления светового сигнала (Start-бит)
    if (isLightPresent()) {
        uint8_t receivedByte = 0;

        // Прием байта с точной математической синхронизацией
        if (receiveByte(receivedByte)) {
            char ch = static_cast<char>(receivedByte);
            processIncomingByte(ch);
        }
    }

    // 3. Отчет о работе каждые 3 секунды (если линия свободна)
    if (liveStreamEnabled && (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL_MS)) {
        lastTelemetryTime = millis();
        printTelemetryReport();
    }

    // 4. Сброс собранного слова по таймауту
    if (bufferIndex > 0 && (millis() - lastCharTime > WORD_FLUSH_TIMEOUT_MS)) {
        flushWordBuffer();
    }
}

// ==========================================
// ФУНКЦИИ СИНХРОНИЗАЦИИ И ПРИЕМА
// ==========================================

/**
 * @brief Проверка наличия света на датчике
 */
bool isLightPresent() {
    int val = analogRead(RX_PIN);
    return (val >= thresholdValue);
}

/**
 * @brief Прецизионный прием одного байта по протоколу UART (8-N-1)
 * 
 * Точный временной расчет сэмплирования:
 * 1. Захват переднего фронта (Start bit).
 * 2. Задержка T/2 (25 мс) -> центр Start-бита. Если свет погас — ложная помеха.
 * 3. Задержка T (50 мс) -> центр Бита 0.
 * 4. Последовательно с шагом T (50 мс) опрашиваем биты 0..7 в их центрах.
 * 5. Задержка T (50 мс) -> центр Стоп-бита (должен быть LOW).
 * 6. Ожидание восстановления оптической линии в темноту перед следующим байтом.
 */
bool receiveByte(uint8_t& outByte) {
    // Шаг 1: Ждем T/2, чтобы попасть в центр стартового бита
    delay(BIT_PERIOD_MS / 2);
    int sample = analogRead(RX_PIN);
    if (sample < (thresholdValue - HYSTERESIS)) {
        // Помеха / шум
        return false;
    }

    // Шаг 2: Переходим к геометрическому центру 1-го бита данных (Bit 0)
    delay(BIT_PERIOD_MS);

    uint8_t reconstructedByte = 0;
    char bitString[9];
    bitString[8] = '\0';

    // Шаг 3: Считываем 8 бит данных строго в центрах битовых интервалов
    for (uint8_t bitIdx = 0; bitIdx < 8; bitIdx++) {
        uint32_t bitStart = millis();

        int bitAdc = analogRead(RX_PIN);
        bool bitVal = (bitAdc >= (thresholdValue - HYSTERESIS));

        if (bitVal) {
            reconstructedByte |= (1 << bitIdx);
            bitString[bitIdx] = '1';
        } else {
            bitString[bitIdx] = '0';
        }

        // Прецизионная выдержка 50 мс между центрами битов
        while (millis() - bitStart < BIT_PERIOD_MS) {
            // NOP
        }
    }

    // Шаг 4: Переход к центру стопового бита
    int stopAdc = analogRead(RX_PIN);
    bool isStopValid = (stopAdc < thresholdValue);

    // Дожидаемся полного окончания стоп-бита (еще T/2 = 25 мс), чтобы не съесть старт-бит следующего символа!
    delay(BIT_PERIOD_MS / 2);

    if (isStopValid) {
        outByte = reconstructedByte;
        
        // Быстрый компактный лог в Serial
        Serial.print(F("[RX] Байт: 0b"));
        Serial.print(bitString);
        Serial.print(F(" | Символ: '"));
        if (reconstructedByte >= 32 && reconstructedByte <= 126) {
            Serial.print((char)reconstructedByte);
        } else if (reconstructedByte == ' ') {
            Serial.print(F(" "));
        } else {
            Serial.print(F("?"));
        }
        Serial.print(F("' (ASCII: "));
        Serial.print(reconstructedByte);
        Serial.println(F(") [OK]"));

        return true;
    } else {
        Serial.print(F("[RX ERROR] Ошибка кадрирования (Stop bit = HIGH, ADC: "));
        Serial.print(stopAdc);
        Serial.println(F(")"));
        return false;
    }
}

// ==========================================
// ОБРАБОТКА ТЕКСТА И БУФЕР
// ==========================================

void processIncomingByte(char ch) {
    lastCharTime = millis();

    // Если пробел или перенос строки — выводим слово
    if (ch == ' ' || ch == '\n' || ch == '\r') {
        if (bufferIndex > 0) {
            flushWordBuffer();
        }
    } else if (ch >= 32 && ch <= 126) {
        if (bufferIndex < BUFFER_SIZE - 1) {
            wordBuffer[bufferIndex++] = ch;
            wordBuffer[bufferIndex] = '\0';
        } else {
            flushWordBuffer();
            wordBuffer[bufferIndex++] = ch;
            wordBuffer[bufferIndex] = '\0';
        }
    }
}

void flushWordBuffer() {
    if (bufferIndex == 0) return;

    Serial.println(F("-------------------------------------------------------"));
    Serial.print(F(">>> ПРИНЯТОЕ СЛОВО / СООБЩЕНИЕ: \""));
    Serial.print(wordBuffer);
    Serial.println(F("\""));
    Serial.println(F("-------------------------------------------------------\n"));

    bufferIndex = 0;
    wordBuffer[0] = '\0';
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
    Serial.print(F(" | Фоновый шум: "));
    Serial.print(ambientNoiseLevel);
    
    if (lightState) {
        Serial.println(F(" | Состояние: [СВЕТ ОБНАРУЖЕН]"));
    } else {
        Serial.println(F(" | Состояние: [РАБОТАЕТ, ОЖИДАНИЕ СИГНАЛА]"));
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
    Serial.print(F(", Пик шума: "));
    Serial.print(maxVal);
    Serial.print(F(" -> Установлен порог: "));
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
            Serial.print(F("[НАСТРОЙКА] Отчет каждые 3 сек: "));
            Serial.println(liveStreamEnabled ? F("ВКЛ") : F("ВЫКЛ"));
        }
    }
}
