/*
 * ============================================================================
 * ПРОЕКТ: Li-Fi Односторонняя оптическая связь (Visible Light Communication)
 * МОДУЛЬ: ПЕРЕДАТЧИК (TX) - СКОРОСТНОЙ РЕЖИМ (50 БОД) + CRC-8
 * ПЛАТФОРМА: Arduino Uno (ATmega328P)
 * СВЕТОДИОД: Модуль LED на Pin 13
 * ============================================================================
 */

#include <Arduino.h>

// ==========================================
// КОНФИГУРАЦИЯ И НАСТРОЙКИ СКОРОСТИ
// ==========================================
constexpr uint8_t TX_PIN = 13;                         // Цифровой пин управления LED
constexpr uint16_t BAUD_RATE = 50;                     // Увеличенная скорость: 50 бит/с (было 20)
constexpr uint32_t BIT_PERIOD_US = 1000000UL / BAUD_RATE; // Длительность бита: 20 000 мкс (20 мс)

// Межсимвольная защитная пауза (20 мс темноты)
constexpr uint32_t INTER_BYTE_DELAY_US = BIT_PERIOD_US;

// ==========================================
// ПРОТОТИПЫ ФУНКЦИЙ
// ==========================================
uint8_t calculateCRC8(const uint8_t* data, size_t len);
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
    Serial.println(F("     >>> Li-Fi ПЕРЕДАТЧИК (TX) [TURBO 50 BAUD] <<<     "));
    Serial.println(F("======================================================="));
    Serial.print(F("[INFO] Скорость Li-Fi: "));
    Serial.print(BAUD_RATE);
    Serial.print(F(" бод (бит/с) | Длительность бита: "));
    Serial.print(BIT_PERIOD_US / 1000);
    Serial.println(F(" мс (Ускорено в 2.5 раза!)"));
    Serial.println(F("[INFO] Протокол: 8-N-1 + аппаратная контрольная сумма CRC-8."));
    Serial.println(F("-------------------------------------------------------"));
    Serial.println(F("Введите фразу в строку ввода Serial Monitor и нажмите Enter:\n"));
}

// ==========================================
// ГЛАВНЫЙ ЦИКЛ (LOOP)
// ==========================================
void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input.length() > 0) {
            uint8_t crc = calculateCRC8(reinterpret_cast<const uint8_t*>(input.c_str()), input.length());

            Serial.println(F("\n>>>>>>>>>>>>>> [ БЫСТРАЯ ПЕРЕДАЧА ] >>>>>>>>>>>>>>"));
            Serial.print(F("[TX] Текст: \""));
            Serial.print(input);
            Serial.println(F("\""));
            Serial.print(F("[TX] Длина: "));
            Serial.print(input.length());
            Serial.print(F(" симв. | CRC-8: 0x"));
            if (crc < 16) Serial.print(F("0"));
            Serial.print(crc, HEX);
            Serial.println();
            Serial.println(F("-------------------------------------------------"));
            
            // Передача текста на повышенной скорости
            sendString(input.c_str());
            
            // Маркер окончания текста
            sendByte('\n');
            
            // Контрольный байт CRC-8
            sendByte(crc);
            
            Serial.println(F("-------------------------------------------------"));
            Serial.println(F("<<<<<<<<<<<<<< [ ПЕРЕДАЧА ЗАВЕРШЕНА ] <<<<<<<<<<<<<<\n"));
        }
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
// СКОРОСТНАЯ ПЕРЕДАЧА ПО ЛУЧУ
// ==========================================

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

    // 4. МЕЖСИМВОЛЬНАЯ ЗАЩИТНАЯ ПАУЗА (LOW)
    uint32_t guardTargetUs = stopTargetUs + INTER_BYTE_DELAY_US;
    while ((long)(micros() - guardTargetUs) < 0);
}

void sendString(const char* str) {
    while (*str) {
        sendByte(static_cast<uint8_t>(*str));
        str++;
    }
}
