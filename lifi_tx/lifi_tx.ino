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

// Межсимвольная пауза (Guard Interval) для идеальной синхронизации длинных слов
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
    digitalWrite(TX_PIN, LOW); // Линия в состоянии покоя (светодиод выключен)

    Serial.begin(115200);
    delay(500);

    Serial.println(F("\n======================================================="));
    Serial.println(F("         >>> Li-Fi ПЕРЕДАТЧИК (TX) ГОТОВ <<<           "));
    Serial.println(F("======================================================="));
    Serial.print(F("[INFO] Скорость: "));
    Serial.print(BAUD_RATE);
    Serial.print(F(" бод (бит/с) | Длительность бита: "));
    Serial.print(BIT_PERIOD_MS);
    Serial.println(F(" мс"));
    Serial.print(F("[INFO] Межсимвольная пауза: "));
    Serial.print(INTER_BYTE_DELAY_MS);
    Serial.println(F(" мс (гарантия приема длинных слов)"));
    Serial.println(F("-------------------------------------------------------"));
    Serial.println(F("Введите любое слово или фразу в монитор порта и нажмите Enter:\n"));
}

// ==========================================
// ГЛАВНЫЙ ЦИКЛ (LOOP)
// ==========================================
void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim(); // Убираем лишние пробелы и \r
        
        if (input.length() > 0) {
            Serial.print(F("[TX] Отправка строки (длина: "));
            Serial.print(input.length());
            Serial.print(F("): \""));
            Serial.print(input);
            Serial.println(F("\""));
            
            // Передача каждого символа строки
            sendString(input.c_str());
            
            // Завершающий пробел в конце строки
            sendByte(' ');
            
            Serial.println(F("[TX] Отправка успешно завершена!\n"));
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
        // Выдержка битового интервала
    }
}

/**
 * @brief Передача байта по протоколу UART (8-N-1) + Межсимвольная защитная пауза
 */
void sendByte(uint8_t data) {
    // 1. СТАРТОВЫЙ БИТ (HIGH - световой импульс)
    sendBit(true);

    // 2. 8 БИТ ДАННЫХ (LSB first - младшим битом вперед)
    for (uint8_t i = 0; i < 8; i++) {
        bool bit = (data >> i) & 0x01;
        sendBit(bit);
    }

    // 3. СТОПОВЫЙ БИТ (LOW - темнота)
    sendBit(false);

    // 4. МЕЖСИМВОЛЬНАЯ ЗАЩИТНАЯ ПАУЗА (LOW)
    // Гарантирует, что RX успеет обработать символ и четко поймать передний фронт следующего
    delay(INTER_BYTE_DELAY_MS);
}

/**
 * @brief Передача строки любой длины
 */
void sendString(const char* str) {
    while (*str) {
        sendByte(static_cast<uint8_t>(*str));
        str++;
    }
}
