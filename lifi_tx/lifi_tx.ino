/*
 * ============================================================================
 * ПРОЕКТ: Li-Fi Односторонняя оптическая связь (Visible Light Communication)
 * МОДУЛЬ: ПЕРЕДАТЧИК (TX) - СТАБИЛЬНЫЙ МАНЧЕСТЕР 25 БОД (IEEE 802.15.7) + CRC
 * ПЛАТФОРМА: Arduino Uno (ATmega328P)
 * СВЕТОДИОД: Модуль LED на Pin 13
 * ============================================================================
 */

#include <Arduino.h>

// ==========================================
// КОНФИГУРАЦИЯ И НАСТРОЙКИ СКОРОСТИ
// ==========================================
constexpr uint8_t TX_PIN = 13;                         // Цифровой пин управления LED
constexpr uint16_t BAUD_RATE = 25;                     // Оптимальная скорость: 25 бит/с
constexpr uint32_t BIT_PERIOD_US = 1000000UL / BAUD_RATE; // 40 000 мкс (40 мс)
constexpr uint32_t HALF_PERIOD_US = BIT_PERIOD_US / 2;     // 20 000 мкс (20 мс на полупериод)

// Межсимвольная защитная пауза (40 мс темноты)
constexpr uint32_t INTER_BYTE_DELAY_US = BIT_PERIOD_US;

// ==========================================
// ПРОТОТИПЫ ФУНКЦИЙ
// ==========================================
uint8_t calculateCRC8(const uint8_t* data, size_t len);
void sendManchesterBit(bool bitVal);
void sendByteManchester(uint8_t data);
void sendStringManchester(const char* str);

// ==========================================
// SETUP
// ==========================================
void setup() {
    pinMode(TX_PIN, OUTPUT);
    digitalWrite(TX_PIN, LOW); // Светодиод выключен в покое

    Serial.begin(115200);
    delay(500);

    Serial.println(F("\n======================================================="));
    Serial.println(F("  >>> Li-Fi ПЕРЕДАТЧИК (TX) [МАНЧЕСТЕР IEEE 802.15.7] <<< "));
    Serial.println(F("======================================================="));
    Serial.print(F("[INFO] Скорость Li-Fi: "));
    Serial.print(BAUD_RATE);
    Serial.print(F(" бод | Длительность бита: "));
    Serial.print(BIT_PERIOD_US / 1000);
    Serial.print(F(" мс (Полутакт: "));
    Serial.print(HALF_PERIOD_US / 1000);
    Serial.println(F(" мс)"));
    Serial.println(F("[INFO] Кодирование: МАНЧЕСТЕР (Без мерцания света)."));
    Serial.println(F("[INFO] Поддержка: РУССКИЙ (UTF-8) и ENGLISH + CRC-8."));
    Serial.println(F("-------------------------------------------------------"));
    Serial.println(F("Введите сообщение и нажмите Enter:\n"));
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

            Serial.println(F("\n>>>>>>>>>>>>>> [ МАНЧЕСТЕР ПЕРЕДАЧА ] >>>>>>>>>>>>>>"));
            Serial.print(F("[TX] Сообщение: \""));
            Serial.print(input);
            Serial.println(F("\""));
            Serial.print(F("[TX] Размер: "));
            Serial.print(input.length());
            Serial.print(F(" байт | CRC-8: 0x"));
            if (crc < 16) Serial.print(F("0"));
            Serial.print(crc, HEX);
            Serial.println();
            Serial.println(F("-------------------------------------------------"));
            
            // 1. Передача текста
            sendStringManchester(input.c_str());
            
            // 2. Маркер окончания текста
            sendByteManchester('\n');
            
            // 3. Байт контрольной суммы CRC-8
            sendByteManchester(crc);
            
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
// МАНЧЕСТЕРСКАЯ ПЕРЕДАЧА
// ==========================================

void sendManchesterBit(bool bitVal) {
    uint32_t startUs = micros();

    // 1-я половина бита
    digitalWrite(TX_PIN, bitVal ? HIGH : LOW);
    while ((long)(micros() - (startUs + HALF_PERIOD_US)) < 0);

    // 2-я половина бита (инверсия)
    digitalWrite(TX_PIN, bitVal ? LOW : HIGH);
    while ((long)(micros() - (startUs + BIT_PERIOD_US)) < 0);
}

void sendByteManchester(uint8_t data) {
    // 1. СИНХРО-БИТ (Манчестер '1' = HIGH -> LOW)
    sendManchesterBit(true);

    // 2. 8 БИТ ДАННЫХ (LSB first)
    for (uint8_t i = 0; i < 8; i++) {
        bool bit = (data >> i) & 0x01;
        sendManchesterBit(bit);
    }

    // 3. Линия в покой (LOW) и межсимвольная пауза
    digitalWrite(TX_PIN, LOW);
    uint32_t guardStartUs = micros();
    while ((long)(micros() - (guardStartUs + INTER_BYTE_DELAY_US)) < 0);
}

void sendStringManchester(const char* str) {
    while (*str) {
        sendByteManchester(static_cast<uint8_t>(*str));
        str++;
    }
}
