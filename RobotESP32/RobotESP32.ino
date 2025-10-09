#include <Arduino.h>
#include <Wire.h> // Biblioteka do komunikacji I2C

// ==================== Podstawowe ustawienia systemowe =====================
#define BAUD 9600 // Prędkość komunikacji ESP-32 -> Komputer

// =================== Zmienne dotyczące komunikacji UART ===================
#define ESP_RX_PIN 16 // RX2(ESP-32) -> PIN_12(Arduino)
#define ESP_TX_PIN 17 // TX2(ESP-32) -> PIN_13(Arduino)
#define UART_BAUD 57600
HardwareSerial SerialPort(2); // Definicja portu szeregowego 2

// ===================== Przyciski sterujące silnikami ======================
#define BTN_X_FWRD 13
#define BTN_X_BACK 12
#define BTN_Y_FWRD 14
#define BTN_Y_BACK 27
#define BTN_Z_FWRD 26
#define BTN_Z_BACK 25
#define BTN_E_FWRD 33
#define BTN_E_BACK 32
#define BTN_MODE 35

// ===================== Zmienne enkoderów (I2C) ======================
const int PCA9548A_ADDR = 0x70; // Adres multipleksera
const int AS5600_ADDR = 0x36; // Natywny adres enkodera
const int AS5600_ZMCO = 0x00; 
const int AS5600_RAW_ANGLE_HIGH = 0x0C; // Rejestry kąta RAW (12 bit)
const int SDA_PIN = 21;
const int SCL_PIN = 22;

const uint8_t ENCODER_CHANNEL[] = {4, 5, 6, 7}; // Kanał na którym znajduje się enkoder
const float ENCODER_LEVER[] = {2, 3.6, 4.5, 4.5}; // Dźwignia ramię/wał silnika
const int16_t ENCODER_ZPOS[] = {2972, 36, 1793, 3824}; // Offset (wartość enkoderów dla pozycji startowej)
int16_t rotationCount[] = {0, 0, 0, 0}; // Liczniki obrotów dla każdej osi
uint16_t lastRawAngle[] = {0, 0, 0, 0}; // Ostatnie odczyty kąta
const float angleConst = 360.0 / 4096.0; // Współczynnik zmiany raw angle na kąt 0-360

// ================== Zmienne do wypisywania stanów ===================
unsigned long previousMillis = 0; // przechowuje czas ostatniego wykonania
const long interval = 1000; // interwał 1000ms


void setup() {
  Serial.begin(BAUD);
  
  // Konfiguracja wszystkich pinów przycisków
  pinMode(BTN_X_FWRD, INPUT_PULLUP);
  pinMode(BTN_X_BACK, INPUT_PULLUP);
  pinMode(BTN_Y_FWRD, INPUT_PULLUP);
  pinMode(BTN_Y_BACK, INPUT_PULLUP);
  pinMode(BTN_Z_FWRD, INPUT_PULLUP);
  pinMode(BTN_Z_BACK, INPUT_PULLUP);
  pinMode(BTN_E_FWRD, INPUT_PULLUP);
  pinMode(BTN_E_BACK, INPUT_PULLUP);
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

  Serial.println("=== SETUP ENDED ===");
}

void loop() {
  if (digitalRead(BTN_MODE) == LOW) {
    /*Serial.write(0x01); // Stan określający moduł przycisków*/
    readButtonsAndControl();
  } else {
    Serial.write(0x02); // Stan określający moduł pozycyjny
    readEncodersAndControl();
  }
  delay(1); 
}

// Sterowanie PRZYCISKAMI
void readButtonsAndControl() {
  byte buttonStates = 0;
  
  // Odczyt stanów przycisków (wciśnięty = LOW) i ustawienie bitów (wciśnięty = 1)
  if (digitalRead(BTN_X_FWRD) == LOW) {buttonStates |= (1 << 0);} // Bit 0: X FWRD (X+)
  if (digitalRead(BTN_X_BACK) == LOW) {buttonStates |= (1 << 1);} // Bit 1: X BACK (X-)
  if (digitalRead(BTN_Y_FWRD) == LOW) {buttonStates |= (1 << 2);} // Bit 2: Y FWRD (Y+)
  if (digitalRead(BTN_Y_BACK) == LOW) {buttonStates |= (1 << 3);} // Bit 3: Y BACK (Y-)
  if (digitalRead(BTN_Z_FWRD) == LOW) {buttonStates |= (1 << 4);} // Bit 4: Z FWRD (Z+)
  if (digitalRead(BTN_Z_BACK) == LOW) {buttonStates |= (1 << 5);} // Bit 5: Z BACK (Z-)
  if (digitalRead(BTN_E_FWRD) == LOW) {buttonStates |= (1 << 6);} // Bit 6: E FWRD (E+)
  if (digitalRead(BTN_E_BACK) == LOW) {buttonStates |= (1 << 7);} // Bit 7: E BACK (E-)
  SerialPort.write(buttonStates); // Wysłanie bajtu do Arduino przez UART2
  
  unsigned long currentMillis = millis(); // aktualny czas
 
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    /*printButtonStates(buttonStates);*/

    for(int i = 0; i < 4; i++){
      uint16_t rawAngle = getEncoderRawAngle(ENCODER_CHANNEL[i]);
      int32_t rawAngleAdjusted = (int32_t)rawAngle - (int32_t)ENCODER_ZPOS[i];
      
      // Modulo dla zakresu 0-4095
      rawAngleAdjusted = ((rawAngleAdjusted % 4096) + 4096) % 4096;
      
      updateRotationCount(i, (uint16_t)rawAngleAdjusted);
      float armAngle = getArmAngle((uint16_t)rawAngleAdjusted, rotationCount[i], ENCODER_LEVER[i]);
      printArmAngle(ENCODER_CHANNEL[i], armAngle);
    }
    Serial.println();
  }
}

// Sterowanie POZYCJĄ
void readEncodersAndControl() {
  delay(500);
}

// =====================================================================================================================================================
// =====================================================================================================================================================
// ================================================================ FUNKCJE DODATKOWE ==================================================================
// =====================================================================================================================================================
// =====================================================================================================================================================

// =========================================================================
// =============================== ENKODERY ================================
// =========================================================================

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
float getArmAngle(uint16_t rawAngle, int16_t rotations, float lever) {
  float totalEncoderAngle = (rotations * 360.0) + (rawAngle * 360.0 / 4096.0); // Całkowity kąt silnika
  return totalEncoderAngle / lever; // Kąt sterowanego ramienia
}
// Funkcja śledzi ilość obrotów dla danego enkodera
void updateRotationCount(uint8_t axisIndex, uint16_t currentRaw) {
  uint16_t lastRaw = lastRawAngle[axisIndex];
  
  // Wykrycie przejścia 4095->0 (obrót w przód)
  if (lastRaw > 3000 && currentRaw < 1000) { rotationCount[axisIndex]++; }
  // Wykrycie przejścia 0->4095 (obrót w tył)
  else if (lastRaw < 1000 && currentRaw > 3000) { rotationCount[axisIndex]--; }
  
  lastRawAngle[axisIndex] = currentRaw;
}

// =========================================================================
// ================================ INNE ===================================
// =========================================================================

void printButtonStates(byte buttonStates) {
  const char* labels[8] = {"X+", "X-", "Y+", "Y-", "Z+", "Z-", "E+", "E-"};
  for (int i = 0; i < 8; ++i) {
    Serial.printf("%s%s:%s", i ? " | " : " ", labels[i], (buttonStates & (1 << i)) ? "ON" : "off");
  }
  Serial.println();
}

void printArmAngle(uint8_t channel, int32_t armAngle) {
  Serial.printf("Kanal %d: %d Raw Angle | ", channel, armAngle);
}

void printArmAngle(uint8_t channel, float armAngle) {
  Serial.printf("Kanal %d: %.2f° | ", channel, armAngle);
}