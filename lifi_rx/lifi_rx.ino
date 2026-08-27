/*
 * ============================================================================
 * ПРОЕКТ: Li-Fi Односторонняя оптическая связь (Visible Light Communication)
 * МОДУЛЬ: ПРИЕМНИК (RX) - МИКРОСЕКУНДНАЯ СИНХРОНИЗАЦИЯ И ЕДИНАЯ СТРОКА
 * ПЛАТФОРМА: Arduino Uno (ATmega328P)
 * ДАТЧИК: Фотодиод BPW24 (Катод -> 5V, Анод -> A0, Резистор 10 кОм -> GND)
 * ============================================================================
 */

#include <Arduino.h>

// ==========================================
// КОНФИГУРАЦИЯ И НАСТРОЙКИ
// ==========================================
constexpr uint8_t RX_PIN = A0;                         // Аналоговый вход фотодиода
constexpr uint16_t BAUD_RATE = 20;                     // Скорость: 20 бит/с
constexpr uint32_t BIT_PERIOD_US = 1000000UL / BAUD_RATE; // Длительность одного бита: 50 000 мкс (50 мс)

// Буфер для сборки предложений любой длины с пробелами
constexpr size_t BUFFER_SIZE = 256;
char sentenceBuffer[BUFFER_SIZE];
size_t bufferIndex = 0;

// Таймаут тишины: 650 мс без новых импульсов = фраза полностью завершена
constexpr uint32_t MESSAGE_TIMEOUT_MS = 650;
uint32_t lastCharTime = 0;
bool isReceivingPhrase = false;

// Порог срабатывания и калибровка
int ambientNoiseLevel = 0;                              // Фоновый уровень темноты
int thresholdValue = 60;                               // Порог переключения (ADC 0..1023)
constexpr int NOISE_MARGIN = 40;                        // Запас над шумом
constexpr int HYSTERESIS = 12;                          // Гистерезис

// ==========================================
// ПРОТОТИПЫ ФУНКЦИЙ
// ==========================================
void calibrateAmbientLight();
bool isLightPresent();
bool receiveByte(uint8_t& outByte);
void processIncomingByte(char ch);
void finalizeSentence();
void handleSerialCommands();

// ==========================================
// SETUP
// ==========================================
void setup() {
    pinMode(RX_PIN, INPUT);

    // Запуск Serial Monitor на 115200 бод
    Serial.begin(115200);
    delay(500);

    Serial.println(F("\n======================================================="));
    Serial.println(F("         >>> Li-Fi ПРИЕМНИК (RX) ЗАПУЩЕН <<<           "));
    Serial.println(F("======================================================="));
    Serial.print(F("[НАСТРОЙКА] Скорость: "));
    Serial.print(BAUD_RATE);
    Serial.print(F(" бод | Длительность бита: "));
    Serial.print(BIT_PERIOD_US / 1000);
    Serial.println(F(" мс"));
    Serial.print(F("[НАСТРОЙКА] Пин фотодиода: A"));
    Serial.println(RX_PIN - A0);
    Serial.println(F("[НАСТРОЙКА] Режим: Тишина в покое. Вывод только при передаче."));

    // Автоматическая калибровка при включении
    calibrateAmbientLight();

    Serial.println(F("-------------------------------------------------------"));
    Serial.println(F("Команды: 'c' - калибровка, '+' / '-' - подстройка порога, 'r' - замер"));
    Serial.println(F("Готов к приему оптического сигнала...\n"));
}

// ==========================================
// ГЛАВНЫЙ ЦИКЛ (LOOP)
// ==========================================
void loop() {
    // 1. Проверка команд от пользователя из монитора порта
    handleSerialCommands();

    // 2. Ожидание появления светового импульса (Start-бит)
    if (isLightPresent()) {
        uint8_t receivedByte = 0;

        // Попытка приема байта с абсолютной микросекундной фазировкой
        if (receiveByte(receivedByte)) {
            char ch = static_cast<char>(receivedByte);
            processIncomingByte(ch);
        }
    }

    // 3. Завершение фразы по таймауту тишины
    if (isReceivingPhrase && (millis() - lastCharTime > MESSAGE_TIMEOUT_MS)) {
        finalizeSentence();
    }
}

// ==========================================
// АЛГОРИТМ ПРИЕМА (МИКРОСЕКУНДНАЯ ТОЧНОСТЬ)
// ==========================================

bool isLightPresent() {
    return (analogRead(RX_PIN) >= thresholdValue);
}

/**
 * @brief Прецизионный прием байта с абсолютной временной привязкой (Zero Cumulative Drift)
 */
bool receiveByte(uint8_t& outByte) {
    uint32_t frameStartUs = micros();

    // Шаг 1: Проверка центра стартового бита (+ 0.5 * T = 25 000 мкс)
    while ((long)(micros() - (frameStartUs + (BIT_PERIOD_US / 2))) < 0) {
        // Ожидание центра старт-бита
    }
    
    int startSample = analogRead(RX_PIN);
    if (startSample < (thresholdValue - HYSTERESIS)) {
        return false; // Ложный шум / кратковременный блик
    }

    uint8_t reconstructedByte = 0;

    // Шаг 2: Считывание 8 бит данных строго в абсолютных точках времени:
    // Бит 0: frameStart + 1.5 * T
    // Бит 1: frameStart + 2.5 * T ...
    // Бит 7: frameStart + 8.5 * T
    for (uint8_t bitIdx = 0; bitIdx < 8; bitIdx++) {
        uint32_t targetUs = frameStartUs + (BIT_PERIOD_US * 3 / 2) + ((uint32_t)bitIdx * BIT_PERIOD_US);
        
        while ((long)(micros() - targetUs) < 0) {
            // Точное ожидание центра бита без накопления ошибки
        }

        int bitAdc = analogRead(RX_PIN);
        if (bitAdc >= (thresholdValue - HYSTERESIS)) {
            reconstructedByte |= (1 << bitIdx);
        }
    }

    // Шаг 3: Проверка стопового бита (frameStart + 9.5 * T)
    uint32_t stopTargetUs = frameStartUs + (BIT_PERIOD_US * 19 / 2);
    while ((long)(micros() - stopTargetUs) < 0) {
        // Ожидание центра стоп-бита
    }

    int stopAdc = analogRead(RX_PIN);
    bool isStopValid = (stopAdc < thresholdValue);

    // Дожидаемся окончания стоп-бита (frameStart + 10.0 * T)
    uint32_t frameEndUs = frameStartUs + (BIT_PERIOD_US * 10);
    while ((long)(micros() - frameEndUs) < 0) {
        // Ожидание конца кадра
    }

    if (isStopValid) {
        outByte = reconstructedByte;
        return true;
    }

    return false;
}

// ==========================================
// ОБРАБОТКА СИМВОЛОВ И ЕДИНАЯ СТРОКА
// ==========================================

void processIncomingByte(char ch) {
    lastCharTime = millis();

    // Если это первый символ новой фразы — печатаем заголовок
    if (!isReceivingPhrase) {
        isReceivingPhrase = true;
        Serial.print(F("[Li-Fi Прием]: "));
    }

    // Обработка символа конца строки
    if (ch == '\n' || ch == '\r') {
        finalizeSentence();
        return;
    }

    // Добавляем символ (включая ПРОБЕЛ) в текущую строку и буфер
    if (ch >= 32 && ch <= 126) {
        Serial.print(ch); // Мгновенный вывод символа в консоль

        if (bufferIndex < BUFFER_SIZE - 1) {
            sentenceBuffer[bufferIndex++] = ch;
            sentenceBuffer[bufferIndex] = '\0';
        }
    }
}

/**
 * @brief Окончание фразы и вывод итогового сообщения
 */
void finalizeSentence() {
    if (!isReceivingPhrase && bufferIndex == 0) return;

    isReceivingPhrase = false;
    Serial.println(); // Перевод строки после потока символов

    if (bufferIndex > 0) {
        Serial.println(F("-------------------------------------------------------"));
        Serial.print(F(">>> [ИТОГОВОЕ СООБЩЕНИЕ]: \""));
        Serial.print(sentenceBuffer);
        Serial.println(F("\""));
        Serial.println(F("-------------------------------------------------------\n"));
    }

    bufferIndex = 0;
    sentenceBuffer[0] = '\0';
}

// ==========================================
// КАЛИБРОВКА И УПРАВЛЕНИЕ
// ==========================================

void calibrateAmbientLight() {
    Serial.println(F("\n[КАЛИБРОВКА] Измерение фонового света..."));

    long sum = 0;
    constexpr int SAMPLES = 80;
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
    thresholdValue = max(ambientNoiseLevel + NOISE_MARGIN, maxVal + 15);
    thresholdValue = constrain(thresholdValue, 25, 950);

    Serial.print(F("[КАЛИБРОВКА] Фон (темнота): "));
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
            thresholdValue = min(1000, thresholdValue + 15);
            Serial.print(F("[РУЧНАЯ НАСТРОЙКА] Порог увеличен: "));
            Serial.println(thresholdValue);
        } else if (cmd == '-') {
            thresholdValue = max(10, thresholdValue - 15);
            Serial.print(F("[РУЧНАЯ НАСТРОЙКА] Порог уменьшен: "));
            Serial.println(thresholdValue);
        } else if (cmd == 'r' || cmd == 'R') {
            int cur = analogRead(RX_PIN);
            Serial.print(F("[ТЕКУЩИЙ ЗАМЕР] АЦП = "));
            Serial.print(cur);
            Serial.print(F(" / 1023 | Порог = "));
            Serial.print(thresholdValue);
            Serial.println((cur >= thresholdValue) ? F(" [СВЕТ ВКЛЮЧЕН]") : F(" [ТЕМНОТА]"));
        }
    }
}
