// ============================================================================
// ПРОЕКТ: Li-Fi Односторонняя оптическая связь (Visible Light Communication)
// МОДУЛЬ: ПРИЕМНИК (RX) - СТАНДАРТ МАНЧЕСТЕР (IEEE 802.15.7) + ТЕЛЕМЕТРИЯ + CRC
// ПЛАТФОРМА: Arduino Uno (ATmega328P)
// ДАТЧИК: Фотодиод BPW24 (Катод -> 5V, Анод -> A0, Резистор -> GND)
// ============================================================================

#include <Arduino.h>

// ==========================================
// КОНФИГУРАЦИЯ И НАСТРОЙКИ СКОРОСТИ
// ==========================================
constexpr uint8_t RX_PIN = A0;                         // Аналоговый вход фотодиода
constexpr uint16_t BAUD_RATE = 40;                     // Скорость: 40 бит/с (эффективная)
constexpr uint32_t BIT_PERIOD_US = 1000000UL / BAUD_RATE; // 25 000 мкс (25 мс)
constexpr uint32_t HALF_PERIOD_US = BIT_PERIOD_US / 2;     // 12 500 мкс (12.5 мс)

// Буфер для сборки UTF-8 текста любой длины
constexpr size_t BUFFER_SIZE = 512;
char sentenceBuffer[BUFFER_SIZE];
size_t bufferIndex = 0;

// Таймаут тишины (окончание всей фразы): 450 мс
constexpr uint32_t MESSAGE_TIMEOUT_MS = 450;
uint32_t lastCharTime = 0;
uint32_t messageStartTime = 0;
bool isReceivingMessage = false;

// Состояние приема пакета
enum RxPacketState {
    STATE_PAYLOAD,
    STATE_WAIT_CRC
};
RxPacketState rxState = STATE_PAYLOAD;

// Динамические параметры адаптивного порога и телеметрии
int ambientNoiseLevel = 3;                              // Уровень темноты
int thresholdValue = 35;                               // Автоматический адаптивный порог
int peakLightAdc = 150;                                // Замеренный пик света
int hysteresisVal = 6;                                 // Гистерезис

// Накопление статистики
long totalLightAdcSum = 0;
int lightSamplesCount = 0;
int manchesterErrorCount = 0;

// ==========================================
// ПРОТОТИПЫ ФУНКЦИЙ
// ==========================================
uint8_t calculateCRC8(const uint8_t* data, size_t len);
void calibrateDarkness();
bool checkStartTrigger();
bool receiveByteManchester(uint8_t& outByte);
void processIncomingByte(uint8_t byteVal);
void finalizeMessageWithCRC(bool hasCrc, uint8_t receivedCRC);
void handleSerialCommands();

// ==========================================
// SETUP
// ==========================================
void setup() {
    pinMode(RX_PIN, INPUT);

    Serial.begin(115200);
    delay(500);

    Serial.println(F("\n============================================================"));
    Serial.println(F("  >>> Li-Fi ПРИЕМНИК (RX) [МАНЧЕСТЕР IEEE 802.15.7 + CRC] <<<"));
    Serial.println(F("============================================================"));
    Serial.print(F("[INFO] Скорость Li-Fi: "));
    Serial.print(BAUD_RATE);
    Serial.print(F(" бод | Длительность бита: "));
    Serial.print(BIT_PERIOD_US / 1000);
    Serial.print(F(" мс (Полутакты: "));
    Serial.print(HALF_PERIOD_US / 1000);
    Serial.println(F(" мс)"));
    Serial.println(F("[INFO] Кодирование: МАНЧЕСТЕР с аппаратным контролем кодовых ошибок."));
    Serial.println(F("[INFO] Языки: РУССКИЙ (UTF-8) и ENGLISH."));

    // Замеряем базовый уровень темноты
    calibrateDarkness();

    Serial.println(F("------------------------------------------------------------"));
    Serial.println(F("Манчестер-декодер запущен и ожидает оптический сигнал...\n"));
}

// ==========================================
// ГЛАВНЫЙ ЦИКЛ (LOOP)
// ==========================================
void loop() {
    handleSerialCommands();

    // Проверка оптического импульса синхро-бита
    if (checkStartTrigger()) {
        uint8_t receivedByte = 0;

        if (receiveByteManchester(receivedByte)) {
            processIncomingByte(receivedByte);
        }
    }

    // Завершение фразы по таймауту
    if (isReceivingMessage && (millis() - lastCharTime > MESSAGE_TIMEOUT_MS)) {
        finalizeMessageWithCRC(false, 0);
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
// МАНЧЕСТЕРСКИЙ ПРИЕМ И ДЕКОДИРОВАНИЕ
// ==========================================

bool checkStartTrigger() {
    int val = analogRead(RX_PIN);
    return (val > (ambientNoiseLevel + 12));
}

// Декодирование 1 Манчестер-байта:
// Бит 1: Первая половина HIGH, Вторая половина LOW
// Бит 0: Первая половина LOW, Вторая половина HIGH
// Нарушение кода (Code Violation): если обе половины одинаковые
bool receiveByteManchester(uint8_t& outByte) {
    uint32_t frameStartUs = micros();

    // 1. СИНХРО-БИТ (Манчестер '1' = HIGH -> LOW):
    // Ждем 1/4 периода (середина 1-й половины) для замера пика света
    uint32_t syncMid1Us = frameStartUs + (BIT_PERIOD_US / 4);
    while ((long)(micros() - syncMid1Us) < 0);

    int startSample = analogRead(RX_PIN);
    if (startSample <= (ambientNoiseLevel + 15)) {
        return false; // Ложная помеха
    }

    // Автоматическая адаптация порога прямо по синхро-биту
    peakLightAdc = startSample;
    thresholdValue = (ambientNoiseLevel + peakLightAdc) / 2;
    hysteresisVal = max(4, (peakLightAdc - ambientNoiseLevel) / 10);

    // Статистика SNR
    totalLightAdcSum += peakLightAdc;
    lightSamplesCount++;

    // Дожидаемся окончания синхро-бита (+ 1.0 * T)
    uint32_t syncEndUs = frameStartUs + BIT_PERIOD_US;
    while ((long)(micros() - syncEndUs) < 0);

    uint8_t reconstructedByte = 0;

    // 2. ДЕКОДИРОВАНИЕ 8 ИНФОРМАЦИОННЫХ БИТОВ МАНЧЕСТЕРА (LSB first)
    for (uint8_t bitIdx = 0; bitIdx < 8; bitIdx++) {
        uint32_t bitStartUs = syncEndUs + ((uint32_t)bitIdx * BIT_PERIOD_US);

        // Замер в центре 1-й половины бита (25% интервала)
        uint32_t sample1Us = bitStartUs + (BIT_PERIOD_US / 4);
        while ((long)(micros() - sample1Us) < 0);
        int val1 = analogRead(RX_PIN);
        bool firstHalfHigh = (val1 >= thresholdValue);

        // Замер в центре 2-й половины бита (75% интервала)
        uint32_t sample2Us = bitStartUs + (BIT_PERIOD_US * 3 / 4);
        while ((long)(micros() - sample2Us) < 0);
        int val2 = analogRead(RX_PIN);
        bool secondHalfHigh = (val2 >= thresholdValue);

        // Правило декодирования Манчестера (IEEE 802.15.7):
        if (firstHalfHigh && !secondHalfHigh) {
            // HIGH -> LOW = Бит 1
            reconstructedByte |= (1 << bitIdx);
        } else if (!firstHalfHigh && secondHalfHigh) {
            // LOW -> HIGH = Бит 0
            // (бит остается 0)
        } else {
            // Нарушение Манчестерского кода (Manchester Code Violation)
            manchesterErrorCount++;
        }
    }

    // 3. Завершение кадра (ожидание конца 8-го бита)
    uint32_t frameEndUs = syncEndUs + (8UL * BIT_PERIOD_US);
    while ((long)(micros() - frameEndUs) < 0);

    outByte = reconstructedByte;
    return true;
}

// ==========================================
// ОБРАБОТКА СИМВОЛОВ И UTF-8
// ==========================================

void processIncomingByte(uint8_t byteVal) {
    if (!isReceivingMessage) {
        isReceivingMessage = true;
        messageStartTime = millis();
        totalLightAdcSum = 0;
        lightSamplesCount = 0;
        manchesterErrorCount = 0;
    }
    lastCharTime = millis();

    // Принят контрольный байт CRC-8
    if (rxState == STATE_WAIT_CRC) {
        finalizeMessageWithCRC(true, byteVal);
        return;
    }

    // Маркер окончания текста
    if (byteVal == '\n' || byteVal == '\r') {
        rxState = STATE_WAIT_CRC;
        return;
    }

    // Вывод символа (ASCII + UTF-8 русские буквы)
    if (byteVal >= 32 && byteVal != 127) {
        Serial.write(byteVal);

        if (bufferIndex < BUFFER_SIZE - 1) {
            sentenceBuffer[bufferIndex++] = static_cast<char>(byteVal);
            sentenceBuffer[bufferIndex] = '\0';
        }
    }
}

// Итоговый отчет: Сообщение + CRC-8 + Телеметрия Манчестера
void finalizeMessageWithCRC(bool hasCrc, uint8_t receivedCRC) {
    isReceivingMessage = false;
    rxState = STATE_PAYLOAD;

    if (bufferIndex == 0) return;

    // Расчет времени передачи
    uint32_t totalDurationMs = lastCharTime - messageStartTime + 250;
    if (totalDurationMs < 50) totalDurationMs = 50;
    float durationSec = totalDurationMs / 1000.0;

    // Расчет скорости
    float bytesPerSec = (float)bufferIndex / durationSec;
    float bitsPerSec = bytesPerSec * 8.0;

    // Расчет контраста и SNR
    int avgLightAdc = (lightSamplesCount > 0) ? (int)(totalLightAdcSum / lightSamplesCount) : peakLightAdc;
    int contrastDelta = avgLightAdc - ambientNoiseLevel;

    float snrDb = 0.0;
    int noiseBase = max(1, ambientNoiseLevel);
    if (avgLightAdc > noiseBase) {
        snrDb = 20.0 * log10((float)avgLightAdc / (float)noiseBase);
    }

    // Расчет CRC-8
    uint8_t calculatedCRC = calculateCRC8(reinterpret_cast<const uint8_t*>(sentenceBuffer), bufferIndex);

    Serial.println();
    Serial.println(F("\n************************************************************"));
    Serial.print(F(">>> [ИТОГОВОЕ СООБЩЕНИЕ]: \""));
    Serial.print(sentenceBuffer);
    Serial.println(F("\""));
    Serial.print(F(">>> [РАЗМЕР СООБЩЕНИЯ]:  "));
    Serial.print(bufferIndex);
    Serial.println(F(" байт"));

    if (hasCrc) {
        Serial.print(F(">>> [КОНТРОЛЬ CRC-8]:     Расчетный = 0x"));
        if (calculatedCRC < 16) Serial.print(F("0"));
        Serial.print(calculatedCRC, HEX);
        Serial.print(F(" | Принятый = 0x"));
        if (receivedCRC < 16) Serial.print(F("0"));
        Serial.println(receivedCRC, HEX);

        if (calculatedCRC == receivedCRC) {
            Serial.println(F(">>> [СТАТУС ЦЕЛОСТНОСТИ]: [УСПЕШНО - ОШИБОК НЕТ!] ✔"));
        } else {
            Serial.println(F(">>> [СТАТУС ЦЕЛОСТНОСТИ]: [ОШИБКА CRC! ДАННЫЕ ИСКАЖЕНЫ] ❌"));
        }
    } else {
        Serial.print(F(">>> [КОНТРОЛЬ CRC-8]:     Таймаут CRC (Расчетный: 0x"));
        if (calculatedCRC < 16) Serial.print(F("0"));
        Serial.print(calculatedCRC, HEX);
        Serial.println(F(")"));
    }

    // МАНЧЕСТЕРСКАЯ ТЕЛЕМЕТРИЯ
    Serial.println(F("------------------------------------------------------------"));
    Serial.println(F(">>> [МЕТРИКИ ОПТИЧЕСКОГО КАНАЛА (МАНЧЕСТЕР IEEE 802.15.7)]:"));
    Serial.print(F("    • Оптический контраст (ΔV): "));
    Serial.print(contrastDelta);
    Serial.print(F(" ADC (Луч: "));
    Serial.print(avgLightAdc);
    Serial.print(F(" | Фон: "));
    Serial.print(ambientNoiseLevel);
    Serial.println(F(")"));

    Serial.print(F("    • SNR (Отношение Сигнал/Шум): "));
    Serial.print(snrDb, 1);
    Serial.print(F(" dB "));
    if (snrDb >= 25.0) {
        Serial.println(F("[ОТЛИЧНЫЙ СИГНАЛ] ★★★"));
    } else if (snrDb >= 16.0) {
        Serial.println(F("[ХОРОШИЙ СИГНАЛ] ★★☆"));
    } else {
        Serial.println(F("[УДОВЛЕТВОРИТЕЛЬНО] ★☆☆"));
    }

    Serial.print(F("    • Ошибки Манчестер-кода:     "));
    Serial.print(manchesterErrorCount);
    Serial.println((manchesterErrorCount == 0) ? F(" (Идеальная форма полутактов) ✔") : F(" (Были сбои формы) ⚠"));

    Serial.print(F("    • Скорость передачи данных:  "));
    Serial.print(bytesPerSec, 1);
    Serial.print(F(" байт/с ("));
    Serial.print(bitsPerSec, 1);
    Serial.println(F(" бит/с)"));

    Serial.print(F("    • Время передачи сообщения:  "));
    Serial.print(durationSec, 1);
    Serial.println(F(" сек"));
    Serial.println(F("************************************************************\n"));

    bufferIndex = 0;
    sentenceBuffer[0] = '\0';
}

// ==========================================
// КАЛИБРОВКА ТЕМНОТЫ И КОМАНДЫ
// ==========================================

void calibrateDarkness() {
    long sum = 0;
    constexpr int SAMPLES = 60;
    int maxVal = 0;

    for (int i = 0; i < SAMPLES; i++) {
        int val = analogRead(RX_PIN);
        sum += val;
        if (val > maxVal) maxVal = val;
        delay(10);
    }

    ambientNoiseLevel = sum / SAMPLES;
    thresholdValue = ambientNoiseLevel + 25;

    Serial.print(F("[КАЛИБРОВКА] Фоновая темнота: "));
    Serial.print(ambientNoiseLevel);
    Serial.println(F(" (ADC 0..1023)\n"));
}

void handleSerialCommands() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'c' || cmd == 'C') {
            calibrateDarkness();
        } else if (cmd == 'r' || cmd == 'R') {
            int cur = analogRead(RX_PIN);
            Serial.print(F("[АЦП]: "));
            Serial.print(cur);
            Serial.print(F(" | Фон: "));
            Serial.print(ambientNoiseLevel);
            Serial.print(F(" | Порог: "));
            Serial.println(thresholdValue);
        }
    }
}
