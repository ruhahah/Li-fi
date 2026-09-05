/*
 * ============================================================================
 * ПРОЕКТ: Li-Fi / Laser Односторонняя оптическая связь
 * МОДУЛЬ: ПЕРЕДАТЧИК (TX) - ПОЛНАЯ ПОДДЕРЖКА КИРИЛЛИЦЫ (UTF-8) + CRC-8
 * ПЛАТФОРМА: Arduino Uno (ATmega328P)
 * ПЕРЕДАТЧИК: Лазерный модуль (KY-008 650нм 5мВт / лазерный диод) на Pin 13
 * ============================================================================
 */

#include <Arduino.h>

// ==========================================
// КОНФИГУРАЦИЯ И НАСТРОЙКИ СКОРОСТИ
// ==========================================
constexpr uint8_t TX_PIN = 13;                         // Цифровой пин управления лазером (KY-008)
constexpr uint16_t BAUD_RATE = 30;                     // Скорость: 30 бит/с (33.3 мс на бит)
constexpr uint32_t BIT_PERIOD_US = 1000000UL / BAUD_RATE; // 33 333 мкс

// Межсимвольная защитная пауза
constexpr uint32_t INTER_BYTE_DELAY_US = BIT_PERIOD_US;

bool isLaserAiming = false;                            // Состояние постоянного свечения лазера для наведения

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
    digitalWrite(TX_PIN, LOW); // Лазер выключен в покое

    Serial.begin(115200);
    delay(500);

    Serial.println(F("\n======================================================="));
    Serial.println(F("  >>> Li-Fi / LASER ПЕРЕДАТЧИК (TX) [РУССКИЙ/ENG + CRC] <<<"));
    Serial.println(F("======================================================="));
    Serial.print(F("[INFO] Скорость Li-Fi:        "));
    Serial.print(BAUD_RATE);
    Serial.print(F(" бод | Длительность бита: "));
    Serial.print(BIT_PERIOD_US / 1000);
    Serial.println(F(" мс"));
    Serial.println(F("[INFO] Источник света:        Лазер (KY-008 650нм 5мВт)"));
    Serial.println(F("[INFO] Поддержка языков:      РУССКИЙ (UTF-8) и ENGLISH."));
    Serial.println(F("[INFO] Контроль целостности:  CRC-8."));
    Serial.println(F("-------------------------------------------------------"));
    Serial.println(F("Команды:"));
    Serial.println(F("  - Введите текст и нажмите Enter для отправки"));
    Serial.println(F("  - Введите 'l' или 'laser' для вкл/выкл лазера (юстировка)\n"));
}

// ==========================================
// ГЛАВНЫЙ ЦИКЛ (LOOP)
// ==========================================
void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input.length() > 0) {
            // Режим юстировки / наведения лазера
            if (input.equalsIgnoreCase("laser") || input.equalsIgnoreCase("l")) {
                isLaserAiming = !isLaserAiming;
                digitalWrite(TX_PIN, isLaserAiming ? HIGH : LOW);
                if (isLaserAiming) {
                    Serial.println(F("\n[ЛАЗЕР]: ВКЛЮЧЕН НЕПРЕРЫВНО для наведения на фотодиод."));
                    Serial.println(F("[ЛАЗЕР]: Направьте луч в центр приемника. Для выключения введите 'l'.\n"));
                } else {
                    Serial.println(F("\n[ЛАЗЕР]: ВЫКЛЮЧЕН. Передатчик готов к отправке текста.\n"));
                }
                return;
            }

            // Если лазер был включен в режиме прицеливания, выключаем перед посылкой
            if (isLaserAiming) {
                isLaserAiming = false;
                digitalWrite(TX_PIN, LOW);
                delay(50);
            }

            // Расчет контрольной суммы от UTF-8 байтов
            uint8_t crc = calculateCRC8(reinterpret_cast<const uint8_t*>(input.c_str()), input.length());

            Serial.println(F("\n>>>>>>>>>>>>>> [ НАЧАЛО ПЕРЕДАЧИ ] >>>>>>>>>>>>>>"));
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
            
            // 1. Передача всех байтов строки (включая русские UTF-8 символы)
            sendString(input.c_str());
            
            // 2. Маркер окончания текста
            sendByte('\n');
            
            // 3. Байт контрольной суммы CRC-8
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
// ПЕРЕДАЧА ПО ЛУЧУ
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
