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

// ===================== Zmienne enkoderów (I2C) ======================
const int PCA9548A_ADDR = 0x70; // Adres multipleksera
const int AS5600_ADDR = 0x36; // Natywny adres enkodera
const int AS5600_ZMCO = 0x00; 
const int AS5600_RAW_ANGLE_HIGH = 0x0C; // Rejestry kąta RAW (12 bit)
const int SDA_PIN = 21;
const int SCL_PIN = 22;
const uint16_t ENCODER_ZPOS[] = {100, 100, 100, 100};

// ================== Zmienne do wypisywania stanów ===================
unsigned long previousMillis = 0; // przechowuje czas ostatniego wykonania
const long interval = 1000; // interwał 500ms


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
  
  // Konfiguracja UART2 dla komunikacji z Arduino
  Serial.println("=== Ustawienia komunikacji UART ===");
  SerialPort.begin(UART_BAUD, SERIAL_8N1, ESP_RX_PIN, ESP_TX_PIN);
  Serial.print("UART RX: ");
  Serial.print(ESP_RX_PIN);
  Serial.print(", TX: ");
  Serial.print(ESP_TX_PIN);
  Serial.print(", Baud: ");
  Serial.print(UART_BAUD);

  // Konfiguracja komunikacji z multiplekserem
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Serial.println("=== SETUP ENDED ===");
}

void loop() {
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
  
  // sprawdź czy minęło 500ms
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    /*printButtonStates(buttonStates);*/

    // Oś E
    uint8_t channel = 1;
    uint16_t rawAngleAdjusted = getEncoderRawAngle(channel) - ENCODER_ZPOS[0];
    printRawAngleAdjusted(channel, rawAngleAdjusted);
    
    // Oś X
    channel = 2;
    rawAngleAdjusted = getEncoderRawAngle(channel) - ENCODER_ZPOS[1];
    printRawAngleAdjusted(channel, rawAngleAdjusted);

    // Oś Y
    channel = 3;
    rawAngleAdjusted = getEncoderRawAngle(channel) - ENCODER_ZPOS[2];
    printRawAngleAdjusted(channel, rawAngleAdjusted);

    //Oś A
    channel = 5;
    rawAngleAdjusted = getEncoderRawAngle(channel) - ENCODER_ZPOS[3];
    printRawAngleAdjusted(channel, rawAngleAdjusted);
  }
  
  delay(1); 
}

// =========================================================================
// ---------------------------FUNKCJE DODATKOWE-----------------------------
// =========================================================================

uint8_t selectI2CChannel(uint8_t channel) {
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  return Wire.endTransmission();
}

uint16_t getEncoderRawAngle(uint8_t channel){
  // Wybór kanalu przez PCA9548A
  uint8_t pca_error = selectI2CChannel(channel);

  if (pca_error != 0) {
    Serial.print("Kanal ");
    Serial.print(channel);
    Serial.print(" -> BLAD MULTIPLEKSERA (Kod: ");
    Serial.print(pca_error);
    Serial.println(").");
    return 0;
  }
  
  uint16_t rawAngle = 0;

  // Sprawdzenie, czy AS5600 odpowiada na 0x36
  Wire.beginTransmission(AS5600_ADDR);
  uint8_t as_error = Wire.endTransmission(true);

  if (as_error == 0) {
    // Odczyt 12-bitowego KĄTA RAW
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_RAW_ANGLE_HIGH); // Wskazujemy rejestr HIGH (0x0C)
    Wire.endTransmission(false);
    
    Wire.requestFrom(AS5600_ADDR, 2); // Odczytujemy 2 bajty (0x0C i 0x0D)
    if (Wire.available() == 2) {
      uint8_t highByte = Wire.read();
      uint8_t lowByte = Wire.read();
      
      // Polaczenie 12 bitow: 4 bity z highByte + 8 bitow z lowByte
      rawAngle = (highByte << 8) | lowByte;
      rawAngle &= 0x0FFF; // Maskowanie do 12 bitow
      
      // Odczyt testowy rejestru ZMCO
      Wire.beginTransmission(AS5600_ADDR);
      Wire.write(AS5600_ZMCO);
      Wire.endTransmission(false);
      Wire.requestFrom(AS5600_ADDR, 1);
      if (Wire.available()) {} 
      else { Serial.println("BLAD ZMCO."); }
    } 
    else {
      Serial.println("BLAD ODCZYTU KĄTA.");
    }
    
  } else {
    Serial.print("Kanal ");
    Serial.print(channel);
    Serial.print(": (AS5600) BRAK (Blad: ");
    Serial.print(as_error);
    Serial.println(").");
  }

  return rawAngle;
}

void printButtonStates(byte buttonStates) {  
  Serial.print(" X+:");
  Serial.print((buttonStates & (1 << 0)) ? "ON" : "off");
  Serial.print(" | X-:");
  Serial.print((buttonStates & (1 << 1)) ? "ON" : "off");
  Serial.print(" | Y+:");
  Serial.print((buttonStates & (1 << 2)) ? "ON" : "off");
  Serial.print(" | Y-:");
  Serial.print((buttonStates & (1 << 3)) ? "ON" : "off");
  Serial.print(" | Z+:");
  Serial.print((buttonStates & (1 << 4)) ? "ON" : "off");
  Serial.print(" | Z-:");
  Serial.print((buttonStates & (1 << 5)) ? "ON" : "off");
  Serial.print(" | E+:");
  Serial.print((buttonStates & (1 << 6)) ? "ON" : "off");
  Serial.print(" | E-:");
  Serial.println((buttonStates & (1 << 7)) ? "ON" : "off");
}

void printRawAngleAdjusted(uint8_t channel, uint16_t rawAngleAdjusted) {
  Serial.print(channel);
  Serial.print(": ");
  Serial.print(rawAngleAdjusted);
  Serial.print("| ");
  Serial.println("");
}