#include <Arduino.h>
#include <Wire.h> // Biblioteka do komunikacji I2C

// ==================== Podstawowe ustawienia systemowe =====================
#define BAUD 9600 // Prędkość komunikacji ESP-32 -> Komputer

// =================== Zmienne dotyczące komunikacji UART ===================
#define ESP_RX_PIN 16 // RX2(ESP-32) -> PIN_12(Arduino)
#define ESP_TX_PIN 17 // TX2(ESP-32) -> PIN_13(Arduino)
#define UART_BAUD 57600
HardwareSerial SerialPort(2); // Definicja portu szeregowego 2
String inputBuffer = "";


// ===================== Przyciski sterujące silnikami ======================
const char* buttonLabels[] = {"X+", "X-", "Y+", "Y-", "Z+", "Z-", "E+", "E-"};
const uint8_t buttonPins[] = { 13,   12,   14,   27,   26,   25,   33,   32};
#define BTN_MODE 35
bool isButtonMode = true;
int currentBtnState;
int lastBtnState = HIGH;

// ===================== Zmienne enkoderów (I2C) ======================
const int PCA9548A_ADDR = 0x70; // Adres multipleksera
const int AS5600_ADDR = 0x36; // Natywny adres enkodera
const int AS5600_ZMCO = 0x00; 
const int AS5600_RAW_ANGLE_HIGH = 0x0C; // Rejestry kąta RAW (12 bit)
#define SDA_PIN 21
#define SCL_PIN 22

const char axisLabels[] = {'E', 'Z', 'Y', 'A'};
const uint8_t ENCODER_CHANNEL[] = {4, 5, 6, 7}; // Kanał na którym znajduje się enkoder [X6,Y7,Z5,E4]
const float ENCODER_LEVER[] = {2, 3.6, 4.5, 4.5}; // Dźwignia (obrotów wału/ramię silnika)
int16_t ENCODER_ZPOS[] = {0, 0, 0, 0}; // Offset (wartość enkoderów dla pozycji startowej)
int16_t rotationCount[] = {0, 0, 0, 0}; // Liczniki obrotów dla każdej osi
uint16_t lastRawAngle[] = {0, 0, 0, 0}; // Ostatnie odczyty kąta
float targetAngles[] = {0.0, 0.0, 0.0, 0.0}; // Oczekiwane kąty
const float angleConst = 360.0 / 4096.0; // Współczynnik zmiany raw angle na kąt 0-360

// ================== Zmienne do wypisywania stanów ===================
unsigned long previousMillis = 0; // przechowuje czas ostatniego wykonania
const long interval = 50; // interwał 1000ms

// =============================== SETUP ================================
void setup() {
    Serial.begin(BAUD);
    
    // Konfiguracja wszystkich pinów przycisków
    for (int i = 0; i < 8; i++) {
        pinMode(buttonPins[i], INPUT_PULLUP);
    }
    pinMode(BTN_MODE, INPUT_PULLUP);
    
    // Konfiguracja UART2 dla komunikacji z Arduino
    Serial.println("=== Ustawienia komunikacji UART ===");
    SerialPort.begin(UART_BAUD, SERIAL_8N1, ESP_RX_PIN, ESP_TX_PIN);
    Serial.print("UART RX: ");
    Serial.print(ESP_RX_PIN);
    Serial.print(", TX: ");
    Serial.print(ESP_TX_PIN);
    Serial.print(", Baud: ");
    Serial.println(UART_BAUD);

    // Konfiguracja komunikacji z multiplekserem
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);

    // Ustalanie pozycji zerowej
    delay(500);
    for(int i = 0; i < 4; i++){
      ENCODER_ZPOS[i] = getEncoderRawAngle(ENCODER_CHANNEL[i]);
      if(ENCODER_ZPOS[i] == 0xFFFF) {
        Serial.printf("BŁĄD: Enkoder na kanale %d nie odpowiada!\n", i);
      }
    }

    Serial.println("=== SETUP ENDED ===");
}

// =============================== LOOP ================================
void loop() {
    // Stworzenie ramki dla obsługi sterowania pozycyjnego - wpisywane w komputerze
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '<') {
            Serial.println("Otrzymano komendę!");
            inputBuffer = ""; // Start nowej ramki np. <X10.5>
        } else if (c == '>') {
            parseCommand(inputBuffer);
        } else {
            inputBuffer += c;
        }
    }

    // Przycisk zmieniający moduł sterowania
    currentBtnState = digitalRead(BTN_MODE);
    if (currentBtnState != lastBtnState) {
        if (currentBtnState == LOW) {
        isButtonMode = !isButtonMode;
        }
        delay(50);
    }
    lastBtnState = currentBtnState;

    // Wybór trybu pracy
    if (isButtonMode) {
        readButtonsAndControl();
    } else {
        readEncodersAndControl();
    }
}

// Sterowanie PRZYCISKAMI
void readButtonsAndControl() {   
    byte buttonStates = 0;
    for (int i = 0; i < 8; i++) {
        if (digitalRead(buttonPins[i]) == LOW) {
            buttonStates |= (1 << i);
        }
    }
    // Wysłanie ramki tekstowej do Arduino
    SerialPort.println("BTN:" + String(buttonStates) + ";");

    
    unsigned long currentMillis = millis(); // aktualny czas
    
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        /*printButtonStates(buttonStates);*/

        for(int i = 0; i < 4; i++){
            // Wyznaczanie kąta dla ramienia
            uint16_t rawAngle = getEncoderRawAngle(ENCODER_CHANNEL[i]); // Pobranie 12-bitowej wartości z enkodera
            int16_t rawAngleAdjusted = rawAngle - ENCODER_ZPOS[i]; // Korekta, aby wyznaczyć pozycję startową
            rawAngleAdjusted = ((rawAngleAdjusted % 4096) + 4096) % 4096; // Normalizacja do przedziału 0-4096
            updateRotationCount(i, rawAngleAdjusted); // Dodawanie pozycji po zrobieniu pełnego okręgu
            float armAngle = getArmAngle(rawAngleAdjusted, rotationCount[i], ENCODER_LEVER[i]); // Obliczenie kąta dla ramienia

            //Serial.printf("Kanal %d: %.3f° - %d Raw | ", ENCODER_CHANNEL[i], armAngle, rawAngle);
            //Serial.printf("Wysłanie ramki: BTN:%s;\n", String(buttonStates));
        }
        //Serial.println();
    }

    delay(5);
}

// Sterowanie POZYCJĄ
void readEncodersAndControl() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    String positionFrame = "POS:"; // Ramka danych do wysłania

    for (int i = 0; i < 4; i++) {
        // Wyznaczanie kąta dla ramienia
        uint16_t rawAngle = getEncoderRawAngle(ENCODER_CHANNEL[i]); // Pobranie 12-bitowej wartości z enkodera
        int16_t rawAngleAdjusted = rawAngle - ENCODER_ZPOS[i]; // Korekta, aby wyznaczyć pozycję startową
        rawAngleAdjusted = ((rawAngleAdjusted % 4096) + 4096) % 4096; // Normalizacja do przedziału 0-4096
        updateRotationCount(i, rawAngleAdjusted); // Dodawanie pozycji po zrobieniu pełnego okręgu
        float armAngle = getArmAngle(rawAngleAdjusted, rotationCount[i], ENCODER_LEVER[i]); // Obliczenie kąta dla ramienia

        // Dodajemy do ramki
        positionFrame += String(axisLabels[i]) + ":" + String(armAngle, 2) + ":" + String(targetAngles[i], 2) + ";";

        /*Serial.printf("Kanal %d: %.2f° - %d Raw (cel: %.2f°) | ", ENCODER_CHANNEL[i], armAngle, rawAngle, targetAngles[i]);*/
    }

    //Serial.println();
    //Serial.println(positionFrame);
    SerialPort.println(positionFrame); // Wysyłamy ramkę do Arduino
    SerialPort.flush(); // Poczekaj aż wszystko się wyśle
    delay(20); // Daj Arduino czas na odbiór
  }
}

// =====================================================================================================================================================
// ================================================================ FUNKCJE DODATKOWE ==================================================================
// =====================================================================================================================================================

// =============================== ENKODERY ================================

// Wybranie kanału na multiplekserze PCA9548A
bool selectI2CChannel(uint8_t channel) {
    Wire.beginTransmission(PCA9548A_ADDR);
    Wire.write(1 << channel); // Ustawienie bitu odpowiadającego kanałowi
    return Wire.endTransmission() == 0; // Zwraca true, jeśli sukces
}
// Sprawdzanie, czy czujnik AS5600 odpowiada na I2C
bool isAS5600Available() {
    Wire.beginTransmission(AS5600_ADDR);
    return Wire.endTransmission(true) == 0;
}
// Odczytanie surowego kąta z AS5600 (12-bitowy)
bool readAS5600Raw(uint16_t &angle) {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_RAW_ANGLE_HIGH); // Rejestr HIGH byte (0x0C)
    if (Wire.endTransmission(false) != 0) return false;

    Wire.requestFrom(AS5600_ADDR, 2); // Odczyt dwóch bajtów: HIGH i LOW
    if (Wire.available() != 2) return false;

    uint8_t high = Wire.read();
    uint8_t low = Wire.read();
    angle = ((high << 8) | low) & 0x0FFF; // Połączenie i maskowanie do 12 bitów
    return true;
}
// Główna funkcja odczytu kąta z enkodera na wybranym kanale
uint16_t getEncoderRawAngle(uint8_t channel) {
    if (!selectI2CChannel(channel)) {
        Serial.printf("Kanał %d -> Błąd multipleksera.\n", channel);
        return 0xFFFF;
    }

    if (!isAS5600Available()) {
        Serial.printf("Kanał %d -> AS5600 niedostępny.\n", channel);
        return 0xFFFF;
    }

    uint16_t angle;
    if (!readAS5600Raw(angle)) {
        Serial.printf("Kanał %d -> Błąd odczytu kąta.\n", channel);
        return 0xFFFF;
    }

    return angle;
}
// Funkcja zwraca kąt o jaki obróciło się ramię
float getArmAngle(int16_t rawAngle, int16_t rotations, float lever) {
    float totalEncoderAngle = (rotations * 360.0) + (rawAngle * angleConst); // Całkowity kąt silnika
    return totalEncoderAngle / lever; // Kąt sterowanego ramienia
}
// Funkcja śledzi ilość obrotów dla danego enkodera
void updateRotationCount(int axisIndex, int16_t currentRaw) {
    int32_t lastRaw = lastRawAngle[axisIndex];
    
    // Wykrycie przejścia 4095->0 (obrót w przód)
    if (lastRaw > 3000 && currentRaw < 1000) { rotationCount[axisIndex]++; }
    // Wykrycie przejścia 0->4095 (obrót w tył)
    else if (lastRaw < 1000 && currentRaw > 3000) { rotationCount[axisIndex]--; }
    
    lastRawAngle[axisIndex] = currentRaw;
}

// ================================ INNE ===================================

void printButtonStates(byte buttonStates) {
    for (int i = 0; i < 8; ++i) {
        Serial.printf("%s%s:%s", i ? " | " : " ", buttonLabels[i], (buttonStates & (1 << i)) ? "ON" : "off");
    }
    Serial.println();
}
void parseCommand(String cmd) {
  if (cmd.length() < 2) return;

  char axis = cmd.charAt(0);
  float angle = cmd.substring(1).toFloat();

  switch (axis) {
    case 'E': targetAngles[0] = angle; break;
    case 'Z': targetAngles[1] = angle; break;
    case 'Y': targetAngles[2] = angle; break;
    case 'A': targetAngles[3] = angle; break;
    default: 
      Serial.printf("Nieznana oś: %c\n", axis);
      return;
  }

  Serial.printf("Otrzymano komendę: oś %c, kąt %.2f° [%s]\n", axis, angle, cmd);
}