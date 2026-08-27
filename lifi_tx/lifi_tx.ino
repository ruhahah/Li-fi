/*
 * ============================================================================
 * ПРОЕКТ: Li-Fi Односторонняя оптическая связь (Visible Light Communication)
 * МОДУЛЬ: ПЕРЕДАТЧИК (TX) - МИКРОСЕКУНДНАЯ ТОЧНОСТЬ ПЕРЕДАЧИ
 * ПЛАТФОРМА: Arduino Uno (ATmega328P)
 * СВЕТОДИОД: Модуль LED на Pin 13
 * ============================================================================
 */

#include <Arduino.h>

// ==========================================
// КОНФИГУРАЦИЯ И НАСТРОЙКИ
// ==========================================
constexpr uint8_t TX_PIN = 13;                         // Цифровой пин управления LED
constexpr uint16_t BAUD_RATE = 20;                     // Скорость передачи: 20 бит/с
constexpr uint32_t BIT_PERIOD_US = 1000000UL / BAUD_RATE; // Длительность бита: 50 000 мкс (50 мс)

// Межсимвольная защитная пауза (50 мс темноты)
constexpr uint32_t INTER_BYTE_DELAY_US = BIT_PERIOD_US;

// ==========================================
// ПРОТОТИПЫ ФУНКЦИЙ
// ==========================================
void sendByte(uint8_t data);
void sendString(const char* str);

// ==========================================
// SETUP
// ==========================================
void setup() {
    pinMode(TX_PIN, OUTPUT);
    digitalWrite(TX_PIN, LOW); // Светодиод выключен в покое

    Serial.begin(115200);
    delay(500);

    Serial.println(F("\n======================================================="));
    Serial.println(F("         >>> Li-Fi ПЕРЕДАТЧИК (TX) ГОТОВ <<<           "));
    Serial.println(F("======================================================="));
    Serial.print(F("[INFO] Скорость Li-Fi: "));
    Serial.print(BAUD_RATE);
    Serial.print(F(" бод | Длительность бита: "));
    Serial.print(BIT_PERIOD_US / 1000);
    Serial.println(F(" мс"));
    Serial.println(F("-------------------------------------------------------"));
    Serial.println(F("Введите предложение с пробелами и нажмите Enter:\n"));
}

// ==========================================
// ГЛАВНЫЙ ЦИКЛ (LOOP)
// ==========================================
void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input.length() > 0) {
            Serial.print(F("\n[TX] Отправка ("));
            Serial.print(input.length());
            Serial.print(F(" симв.): \""));
            Serial.print(input);
            Serial.println(F("\""));
            
            // Передача всех символов (включая пробелы)
            sendString(input.c_str());
            
            // Символ переноса строки как маркер завершения фразы
            sendByte('\n');
            
            Serial.println(F("[TX] Отправка успешно завершена!\n"));
        }
    }
}

// ==========================================
// РЕАЛИЗАЦИЯ ПЕРЕДАЧИ
// ==========================================

/**
 * @brief Прецизионная отправка байта с абсолютной микросекундной фазировкой
 */
void sendByte(uint8_t data) {
    uint32_t frameStartUs = micros();

    // 1. СТАРТОВЫЙ БИТ (HIGH)
    digitalWrite(TX_PIN, HIGH);
    while ((long)(micros() - (frameStartUs + BIT_PERIOD_US)) < 0);

    // 2. 8 БИТ ДАННЫХ (LSB first)
    for (uint8_t i = 0; i < 8; i++) {
        bool bit = (data >> i) & 0x01;
        digitalWrite(TX_PIN, bit ? HIGH : LOW);
        
        uint32_t targetUs = frameStartUs + ((uint32_t)(i + 2) * BIT_PERIOD_US);
        while ((long)(micros() - targetUs) < 0);
    }

    // 3. СТОПОВЫЙ БИТ (LOW)
    digitalWrite(TX_PIN, LOW);
    uint32_t stopTargetUs = frameStartUs + (10UL * BIT_PERIOD_US);
    while ((long)(micros() - stopTargetUs) < 0);

    // 4. МЕЖСИМВОЛЬНАЯ ПАУЗА (LOW)
    uint32_t guardTargetUs = stopTargetUs + INTER_BYTE_DELAY_US;
    while ((long)(micros() - guardTargetUs) < 0);
}

/**
 * @brief Передача строки
 */
void sendString(const char* str) {
    while (*str) {
        sendByte(static_cast<uint8_t>(*str));
        str++;
    }
}
