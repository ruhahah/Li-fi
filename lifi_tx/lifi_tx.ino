/*
 * ============================================================================
 * ПРОЕКТ: Li-Fi Односторонняя оптическая связь (Visible Light Communication)
 * МОДУЛЬ: ПЕРЕДАТЧИК (TX) - С КОНТРОЛЬНОЙ СУММОЙ CRC-8
 * ПЛАТФОРМА: Arduino Uno (ATmega328P)
 * СВЕТОДИОД: Модуль LED на Pin 13
 * ============================================================================
 */

#include <Arduino.h>

// ==========================================
// КОНФИГУРАЦИЯ И НАСТРОЙКИ
// ==========================================
constexpr uint8_t TX_PIN = 13;                         // Цифровой пин управления LED
constexpr uint16_t BAUD_RATE = 20;                     // Скорость: 20 бит/с
constexpr uint32_t BIT_PERIOD_US = 1000000UL / BAUD_RATE; // Длительность одного бита: 50 000 мкс (50 мс)

// Межсимвольная защитная пауза (50 мс темноты)
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
    Serial.println(F("     >>> Li-Fi ПЕРЕДАТЧИК (TX) [С ЗАЩИТОЙ CRC-8] <<<   "));
    Serial.println(F("======================================================="));
    Serial.print(F("[INFO] Скорость Li-Fi: "));
    Serial.print(BAUD_RATE);
    Serial.print(F(" бод | Длительность бита: "));
    Serial.print(BIT_PERIOD_US / 1000);
    Serial.println(F(" мс"));
    Serial.println(F("[INFO] В конце каждого сообщения отправляется байт CRC-8."));
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
            // 1. Расчет контрольной суммы CRC-8
            uint8_t crc = calculateCRC8(reinterpret_cast<const uint8_t*>(input.c_str()), input.length());

            Serial.println(F("\n>>>>>>>>>>>>>> [ НАЧАЛО ПЕРЕДАЧИ ] >>>>>>>>>>>>>>"));
            Serial.print(F("[TX] Сообщение: \""));
            Serial.print(input);
            Serial.println(F("\""));
            Serial.print(F("[TX] Длина: "));
            Serial.print(input.length());
            Serial.print(F(" симв. | Контрольная сумма CRC-8: 0x"));
            if (crc < 16) Serial.print(F("0"));
            Serial.print(crc, HEX);
            Serial.print(F(" (BIN: 0b"));
            for (int i = 7; i >= 0; i--) Serial.print((crc >> i) & 1);
            Serial.println(F(")"));
            Serial.println(F("-------------------------------------------------"));
            
            // 2. Передача текста по оптическому лучу
            sendString(input.c_str());
            
            // 3. Отправка маркера окончания текста ('\n')
            sendByte('\n');
            
            // 4. Отправка контрольного байта CRC-8
            sendByte(crc);
            
            Serial.println(F("-------------------------------------------------"));
            Serial.println(F("<<<<<<<<<<<<<< [ ПЕРЕДАЧА ЗАВЕРШЕНА ] <<<<<<<<<<<<<<\n"));
        }
    }
}

// ==========================================
// РАСЧЕТ CRC-8 (Полином 0x07: x^8 + x^2 + x + 1)
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
// РЕАЛИЗАЦИЯ ОПТИЧЕСКОЙ ПЕРЕДАЧИ
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

    // 4. МЕЖСИМВОЛЬНАЯ ЗАЩИТНАЯ ПАУЗА (LOW)
    uint32_t guardTargetUs = stopTargetUs + INTER_BYTE_DELAY_US;
    while ((long)(micros() - guardTargetUs) < 0);
}

/**
 * @brief Передача нуль-терминированной строки
 */
void sendString(const char* str) {
    while (*str) {
        sendByte(static_cast<uint8_t>(*str));
        str++;
    }
}
