/*
 * ============================================================================
 * ПРОЕКТ: Li-Fi Односторонняя оптическая связь (Visible Light Communication)
 * МОДУЛЬ: ПЕРЕДАТЧИК (TX) - С НАДЕЖНОЙ МЕЖСИМВОЛЬНОЙ СИНХРОНИЗАЦИЕЙ
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
constexpr uint32_t BIT_PERIOD_MS = 1000 / BAUD_RATE;   // Длительность одного бита: 50 мс

// Межсимвольная пауза (Guard Interval) для надежной синхронизации
constexpr uint32_t INTER_BYTE_DELAY_MS = BIT_PERIOD_MS; // 50 мс темноты между символами

// ==========================================
// ПРОТОТИПЫ ФУНКЦИЙ
// ==========================================
void sendBit(bool bitVal);
void sendByte(uint8_t data);
void sendString(const char* str);

// ==========================================
// SETUP
// ==========================================
void setup() {
    pinMode(TX_PIN, OUTPUT);
    digitalWrite(TX_PIN, LOW); // Линия в покое (LED выключен)

    Serial.begin(115200);
    delay(500);

    Serial.println(F("\n======================================================="));
    Serial.println(F("         >>> Li-Fi ПЕРЕДАТЧИК (TX) ГОТОВ <<<           "));
    Serial.println(F("======================================================="));
    Serial.print(F("[INFO] Скорость Li-Fi: "));
    Serial.print(BAUD_RATE);
    Serial.print(F(" бод (бит/с) | Бит: "));
    Serial.print(BIT_PERIOD_MS);
    Serial.println(F(" мс"));
    Serial.print(F("[INFO] Межсимвольная пауза: "));
    Serial.print(INTER_BYTE_DELAY_MS);
    Serial.println(F(" мс"));
    Serial.println(F("-------------------------------------------------------"));
    Serial.println(F("Введите фразу из нескольких слов с пробелами и нажмите Enter:\n"));
}

// ==========================================
// ГЛАВНЫЙ ЦИКЛ (LOOP)
// ==========================================
void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input.length() > 0) {
            Serial.print(F("\n[TX] Отправка сообщения (длина: "));
            Serial.print(input.length());
            Serial.print(F(" симв.): \""));
            Serial.print(input);
            Serial.println(F("\""));
            
            // 1. Передача всех символов строки (включая пробелы между словами)
            sendString(input.c_str());
            
            // 2. Отправка символа переноса строки (\n) как маркер ПОЛНОЙ ОСТАНОВКИ
            sendByte('\n');
            
            Serial.println(F("[TX] Сообщение успешно отправлено!\n"));
        }
    }
}

// ==========================================
// РЕАЛИЗАЦИЯ ПЕРЕДАЧИ
// ==========================================

/**
 * @brief Прецизионная передача одного бита
 */
void sendBit(bool bitVal) {
    uint32_t startTime = millis();
    digitalWrite(TX_PIN, bitVal ? HIGH : LOW);
    
    while (millis() - startTime < BIT_PERIOD_MS) {
        // Ожидание периода бита
    }
}

/**
 * @brief Передача байта по протоколу UART (8-N-1) + Защитная пауза
 */
void sendByte(uint8_t data) {
    // 1. СТАРТОВЫЙ БИТ (HIGH - световой импульс)
    sendBit(true);

    // 2. 8 БИТ ДАННЫХ (LSB first)
    for (uint8_t i = 0; i < 8; i++) {
        bool bit = (data >> i) & 0x01;
        sendBit(bit);
    }

    // 3. СТОПОВЫЙ БИТ (LOW - темнота)
    sendBit(false);

    // 4. МЕЖСИМВОЛЬНАЯ ЗАЩИТНАЯ ПАУЗА (LOW)
    delay(INTER_BYTE_DELAY_MS);
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
