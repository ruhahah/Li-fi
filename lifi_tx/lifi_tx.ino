/*
 * ============================================================================
 * ПРОЕКТ: Li-Fi Односторонняя оптическая связь (Visible Light Communication)
 * МОДУЛЬ: ПЕРЕДАТЧИК (TX)
 * ПЛАТФОРМА: Arduino Uno (ATmega328P)
 * СВЕТОДИОД: KY-009 / Любой LED на Pin 13 (GND -> GND, Signal -> D13)
 * ============================================================================
 */

#include <Arduino.h>

// ==========================================
// КОНФИГУРАЦИЯ И НАСТРОЙКИ
// ==========================================
constexpr uint8_t TX_PIN = 13;               // Цифровой пин управления LED
constexpr uint16_t BAUD_RATE = 20;           // Скорость передачи: 20 бит/с
constexpr uint32_t BIT_PERIOD_MS = 1000 / BAUD_RATE; // Длительность одного бита: 50 мс
constexpr uint32_t WORD_PAUSE_MS = 300;      // Пауза между отправкой строк (мс)

// Тестовое сообщение для циклической передачи (если в Serial ничего не ввели)
const char DEFAULT_MESSAGE[] = "Hello Li-Fi World! ";

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
    // Настройка пина TX на выход
    pinMode(TX_PIN, OUTPUT);
    digitalWrite(TX_PIN, LOW); // Линия в состоянии покоя (свет выключен)

    // Инициализация аппаратного UART для монитора порта (управление и логи)
    Serial.begin(115200);
    while (!Serial && millis() < 2000); // Ожидание подключения Serial Monitor

    Serial.println(F("========================================"));
    Serial.println(F("    Li-Fi TRANSMITTER (TX) INITIALIZED  "));
    Serial.println(F("========================================"));
    Serial.print(F("Speed: "));
    Serial.print(BAUD_RATE);
    Serial.print(F(" baud | Bit duration: "));
    Serial.print(BIT_PERIOD_MS);
    Serial.println(F(" ms"));
    Serial.println(F("Frame format: 1 Start (HIGH) + 8 Data Bits (LSB first) + 1 Stop (LOW)"));
    Serial.println(F("Type text into Serial Monitor and press Enter to send,"));
    Serial.println(F("or wait for automatic periodic transmission."));
    Serial.println(F("========================================\n"));
}

// ==========================================
// ГЛАВНЫЙ ЦИКЛ (LOOP)
// ==========================================
void loop() {
    // 1. Проверка ввода из Serial Monitor
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim(); // Удаляем пробелы и переносы на концах
        if (input.length() > 0) {
            Serial.print(F("[TX Send User]: "));
            Serial.println(input);
            sendString(input.c_str());
            // Отправляем пробел в конце, чтобы приемник завершил слово
            sendByte(' ');
        }
    } 
    // 2. Если пользователь ничего не вводит, отправляем демонстрационную строку
    else {
        Serial.print(F("[TX Auto]: "));
        Serial.println(DEFAULT_MESSAGE);
        sendString(DEFAULT_MESSAGE);
        delay(WORD_PAUSE_MS);
    }
}

// ==========================================
// РЕАЛИЗАЦИЯ ФУНКЦИЙ ПЕРЕДАЧИ
// ==========================================

/**
 * @brief Передача одного бита с точной выдержкой интервала времени.
 * @param bitVal Логическое значение бита (true = HIGH / LED ON, false = LOW / LED OFF).
 */
void sendBit(bool bitVal) {
    uint32_t startTime = millis();
    
    // Модуляция OOK (On-Off Keying)
    digitalWrite(TX_PIN, bitVal ? HIGH : LOW);
    
    // Блокирующая прецизионная задержка для сохранения стабильной фазы
    while (millis() - startTime < BIT_PERIOD_MS) {
        // NOP (ждем завершения периода бита)
    }
}

/**
 * @brief Передача одного байта (символа) по протоколу 8-N-1 (с инверсным Idle = LOW).
 * 
 * Структура оптического кадра:
 * 1. START BIT = HIGH (1 период): вспышка света сигнализирует приемнику о начале кадра.
 * 2. 8 DATA BITS (LSB first): передача битов от младшего (bit 0) к старшему (bit 7).
 * 3. STOP BIT = LOW (1 период): выключение света для фиксации конца кадра и сброса линии.
 * 
 * @param data Передаваемый байт данных (ASCII символ).
 */
void sendByte(uint8_t data) {
    // 1. СТАРТОВЫЙ БИТ (HIGH - свет включается)
    sendBit(true);

    // 2. 8 БИТ ДАННЫХ (младшим битом вперед - LSB first)
    for (uint8_t i = 0; i < 8; i++) {
        bool bit = (data >> i) & 0x01;
        sendBit(bit);
    }

    // 3. СТОПОВЫЙ БИТ (LOW - свет выключается)
    sendBit(false);
}

/**
 * @brief Передача нуль-терминированной строки.
 * @param str Указатель на C-строку.
 */
void sendString(const char* str) {
    while (*str) {
        sendByte(static_cast<uint8_t>(*str));
        str++;
    }
}
