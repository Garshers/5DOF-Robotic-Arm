#include <Arduino.h>
#include <SoftwareSerial.h>

// ==================== Podstawowe ustawienia systemowe =====================
#define BAUD 9600 // Prędkość komunikacji Arudino -> Komputer

// =================== Zmienne dotyczące komunikacji UART ===================
#define UART_RX_PIN 13 // PIN_13(Arduino) -> TX2(ESP-32)
#define UART_TX_PIN 12 // PIN_12(Arduino) -> RX2(ESP-32)
#define UART_BAUD 57600 // Szybkość transmisji
byte buttonStates = 0; // Zmienna przechowująca stany przycisków
SoftwareSerial esp32Serial(UART_RX_PIN, UART_TX_PIN); // Definicja portu szeregowego

// ======================= Zmienne sterujące silnikami ======================
#define STEP_X 2
#define STEP_Y 3
#define STEP_Z 4
#define DIR_X 5
#define DIR_Y 6
#define DIR_Z 7
#define LIMIT_X_PIN 9
#define LIMIT_Y_PIN 10
#define LIMIT_Z_PIN 11

// Silnik piąty - podpięty osobno - NEMA 14
#define STEP_E A1
#define DIR_E A0
#define ENABLE_E A2
#define LIMIT_E_PIN A3

const unsigned int STEP_DELAY_MICROS = 1000; // Szybkość kroku [ms]
bool limit_Z_hit = false; // Zmienne do wykrywania zbocza na krańcówce Z
bool limit_Z_last = false;
bool lockedDirZ = false; // Kierunek w momencie naciśnięcia krańcówki Osi Z

// Funkcja sterująca pojedynczym krokiem
void stepPulse(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(STEP_DELAY_MICROS);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(STEP_DELAY_MICROS);
}

void setup() {
  pinMode(STEP_X, OUTPUT);
  pinMode(DIR_X, OUTPUT);
  pinMode(STEP_Y, OUTPUT);
  pinMode(DIR_Y, OUTPUT);
  pinMode(STEP_Z, OUTPUT);
  pinMode(DIR_Z, OUTPUT);

  pinMode(LIMIT_X_PIN, INPUT_PULLUP);
  pinMode(LIMIT_Y_PIN, INPUT_PULLUP);
  pinMode(LIMIT_Z_PIN, INPUT_PULLUP);

  Serial.begin(BAUD);

  Serial.println("=== Ustawienia komunikacji UART ===");
  esp32Serial.begin(UART_BAUD); // Ustawienie prędkości komunikacji Arduino z ESP-32 [Musi być taka sama na obu urządzeniach]
  Serial.print("UART RX: ");
  Serial.print(UART_RX_PIN);
  Serial.print(", TX: ");
  Serial.print(UART_TX_PIN);
  Serial.print(", Baud: ");
  Serial.print(UART_BAUD);
  Serial.println(". Czekam na dane...");
}

void loop() {
  if (esp32Serial.available() > 0) {
    buttonStates = esp32Serial.read(); // Odbieranie danych z ESP-32
    /*printButtonStates(buttonStates);*/
  }
  
  // Wydobycie stanów przycisków z odebranego bajtu
  bool BTN_X_FRWD = (buttonStates & (1 << 0));
  bool BTN_X_BACK = (buttonStates & (1 << 1));
  bool BTN_Y_FRWD = (buttonStates & (1 << 2));
  bool BTN_Y_BACK = (buttonStates & (1 << 3));
  bool BTN_Z_FRWD = (buttonStates & (1 << 4));
  bool BTN_Z_BACK = (buttonStates & (1 << 5));
  bool BTN_E_FRWD = (buttonStates & (1 << 6));
  bool BTN_E_BACK = (buttonStates & (1 << 7));
  
  // Odczyt stanów krańcówek
  bool limit_E_hit = (digitalRead(LIMIT_E_PIN) == HIGH);
  bool limit_X_hit = (digitalRead(LIMIT_X_PIN) == HIGH);
  bool limit_Y_hit = (digitalRead(LIMIT_Y_PIN) == HIGH);

  // Detekcja zbocza dla krańcówki osi Z i zapamiętanie kierunku
  limit_Z_hit = (digitalRead(LIMIT_Z_PIN) == HIGH);
  if (limit_Z_hit == true && limit_Z_last == false) {
    Serial.println("!!! Naciśnięto krańcówkę osi Z (Detekcja zbocza)!");
    if(BTN_Z_FRWD) {lockedDirZ = LOW;}
    if(BTN_Z_BACK) {lockedDirZ = HIGH;}
  }
  limit_Z_last = limit_Z_hit;
  
  // =========================================================================
  // ---------------------------STEROWANIE SILNIKAMI--------------------------
  // =========================================================================

  // Oś E
  if (BTN_E_FRWD) {
    digitalWrite(DIR_E, LOW);
    stepPulse(STEP_E);
  }
  if (BTN_E_BACK && !limit_E_hit) {
    digitalWrite(DIR_E, HIGH);
    stepPulse(STEP_E);
  }

  // Oś X
  if (BTN_X_FRWD) {
    digitalWrite(DIR_X, LOW);
    stepPulse(STEP_X);
  }
  if (BTN_X_BACK && !limit_X_hit) {
    digitalWrite(DIR_X, HIGH);
    stepPulse(STEP_X);
  }

  // Oś Y
  if (BTN_Y_FRWD) {
    digitalWrite(DIR_Y, LOW);
    stepPulse(STEP_Y);
  }
  if (BTN_Y_BACK && !limit_Y_hit) {
    digitalWrite(DIR_Y, HIGH);
    stepPulse(STEP_Y);
  }

  // Oś Z
  if ((BTN_Z_FRWD && !limit_Z_hit) || (limit_Z_hit && lockedDirZ != LOW)) {
    digitalWrite(DIR_Z, LOW);
    stepPulse(STEP_Z);
  }
  if ((BTN_Z_BACK && !limit_Z_hit) || (limit_Z_hit && lockedDirZ != HIGH)) {
    digitalWrite(DIR_Z, HIGH);
    stepPulse(STEP_Z);
  }
}


// =========================================================================
// ---------------------------FUNKCJE DODATKOWE-----------------------------
// =========================================================================

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