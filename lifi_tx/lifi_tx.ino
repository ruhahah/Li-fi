/*
 * ============================================================================
 * ПРОЕКТ: Li-Fi Односторонняя оптическая связь (Visible Light Communication)
 * МОДУЛЬ: ПЕРЕДАТЧИК (TX) С ДЕТАЛЬНОЙ ВИЗУАЛИЗАЦИЕЙ ОТПРАВКИ
 * ПЛАТФОРМА: Arduino Uno (ATmega328P)
 * СВЕТОДИОД: Светодиод / модуль на Pin 13 (GND -> GND, Signal -> D13)
 * ============================================================================
 */

#include <Arduino.h>

// ==========================================
// КОНФИГУРАЦИЯ И НАСТРОЙКИ
// ==========================================
constexpr uint8_t TX_PIN = 13;               // Цифровой пин управления LED
constexpr uint16_t BAUD_RATE = 20;           // Скорость передачи: 20 бит/с
constexpr uint32_t BIT_PERIOD_MS = 1000 / BAUD_RATE; // Длительность одного бита: 50 мс

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
    digitalWrite(TX_PIN, LOW); // Линия в состоянии покоя (свет выключен)

    Serial.begin(115200);
    while (!Serial && millis() < 2000);

    Serial.println(F("\n======================================================="));
    Serial.println(F("       Li-Fi TRANSMITTER (TX) - READY TO TRANSMIT      "));
    Serial.println(F("======================================================="));
    Serial.print(F("[CONFIG] Speed: "));
    Serial.print(BAUD_RATE);
    Serial.print(F(" baud | Bit duration: "));
    Serial.print(BIT_PERIOD_MS);
    Serial.println(F(" ms"));
    Serial.println(F("[CONFIG] TX Pin: D13 (LED)"));
    Serial.println(F("[CONFIG] Frame format: START(1) + 8 DATA BITS (LSB) + STOP(0)"));
    Serial.println(F("-------------------------------------------------------"));
    Serial.println(F("Введите текст в строку ввода Serial Monitor и нажмите Enter:"));
    Serial.println(F("=======================================================\n"));
}

// ==========================================
// ГЛАВНЫЙ ЦИКЛ (LOOP)
// ==========================================
void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input.length() > 0) {
            Serial.println(F("\n>>>>>>>>>>>> [ НАЧАЛО ПЕРЕДАЧИ ] >>>>>>>>>>>>"));
            Serial.print(F("[TX ТЕКСТ]: \""));
            Serial.print(input);
            Serial.print(F("\" (Длина: "));
            Serial.print(input.length());
            Serial.println(F(" симв.)"));
            Serial.println(F("---------------------------------------------"));
            
            // Передача каждого символа по оптическому каналу
            sendString(input.c_str());
            
            // Завершающий пробел для сброса буфера на RX
            sendByte(' ');
            
            Serial.println(F("---------------------------------------------"));
            Serial.println(F("<<<<<<<<<<<< [ ПЕРЕДАЧА ЗАВЕРШЕНА ] <<<<<<<<<<<<\n"));
            Serial.println(F("Готов к следующему сообщению...\n"));
        }
    }
}

// ==========================================
// РЕАЛИЗАЦИЯ ФУНКЦИЙ ПЕРЕДАЧИ
// ==========================================

/**
 * @brief Передача одного бита с точной выдержкой интервала времени.
 */
void sendBit(bool bitVal) {
    uint32_t startTime = millis();
    digitalWrite(TX_PIN, bitVal ? HIGH : LOW);
    
    while (millis() - startTime < BIT_PERIOD_MS) {
        // Прецизионное ожидание периода бита
    }
}

/**
 * @brief Передача одного байта с подробным логом в Serial Monitor.
 */
void sendByte(uint8_t data) {
    char ch = static_cast<char>(data);
    
    Serial.print(F("[TX Символ]: '"));
    if (ch >= 32 && ch <= 126) {
        Serial.print(ch);
    } else {
        Serial.print(F("SPC"));
    }
    Serial.print(F("' | HEX: 0x"));
    if (data < 16) Serial.print(F("0"));
    Serial.print(data, HEX);
    Serial.print(F(" | Биты: [START:1] -> "));

    // 1. СТАРТОВЫЙ БИТ (HIGH)
    sendBit(true);

    // 2. 8 БИТ ДАННЫХ (LSB first)
    for (uint8_t i = 0; i < 8; i++) {
        bool bit = (data >> i) & 0x01;
        Serial.print(bit ? '1' : '0');
        Serial.print(F(" "));
        sendBit(bit);
    }

    // 3. СТОПОВЫЙ БИТ (LOW)
    sendBit(false);
    Serial.println(F("-> [STOP:0] OK"));
}

/**
 * @brief Передача строки.
 */
void sendString(const char* str) {
    while (*str) {
        sendByte(static_cast<uint8_t>(*str));
        str++;
    }
}
