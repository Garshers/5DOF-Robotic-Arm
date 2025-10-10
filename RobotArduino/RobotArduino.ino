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

enum ControlMode { MODE_NONE, MODE_BUTTONS, MODE_POSITION };
ControlMode currentMode = MODE_NONE;

const unsigned int STEP_DELAY_MICROS = 1000; // Szybkość kroku [µs]
bool limit_Z_last = false; // Zmienna do wykrywania zbocza na krańcówce Z
int lastDirZ = 0; // Ostatni kierunek ruchu Z (1 = FRWD/LOW, 2 = BACK/HIGH, 0 = brak ruchu)
bool limit_Z_blocked = false; // Flaga blokady kierunku

// ======================== Sterowanie pozycyjne ========================
float currentAngles[4] = {0.0, 0.0, 0.0, 0.0};
float targetAngles[4] = {10.0, 0.0, 0.0, 0.0};
unsigned long lastStepTime[4] = {0, 0, 0, 0};
unsigned long lastPositionUpdate = 0; // Timeout dla trybu pozycyjnego
const unsigned long POSITION_TIMEOUT = 3000; // 3 sekundy

void setup() {
    pinMode(STEP_X, OUTPUT);
    pinMode(DIR_X, OUTPUT);
    pinMode(STEP_Y, OUTPUT);
    pinMode(DIR_Y, OUTPUT);
    pinMode(STEP_Z, OUTPUT);
    pinMode(DIR_Z, OUTPUT);
    pinMode(STEP_E, OUTPUT);
    pinMode(DIR_E, OUTPUT);
    pinMode(ENABLE_E, OUTPUT);

    pinMode(LIMIT_X_PIN, INPUT_PULLUP);
    pinMode(LIMIT_Y_PIN, INPUT_PULLUP);
    pinMode(LIMIT_Z_PIN, INPUT_PULLUP);
    pinMode(LIMIT_E_PIN, INPUT_PULLUP);

    digitalWrite(ENABLE_E, LOW); // Włącz silnik E (LOW = aktywny dla większości sterowników)

    Serial.begin(BAUD);

    Serial.println("=== Ustawienia komunikacji UART ===");
    esp32Serial.begin(UART_BAUD);
    Serial.print("UART RX: ");
    Serial.print(UART_RX_PIN);
    Serial.print(", TX: ");
    Serial.print(UART_TX_PIN);
    Serial.print(", Baud: ");
    Serial.print(UART_BAUD);
    Serial.println(". Czekam na dane...");
}

void loop() {
    if (esp32Serial.available()) {
        String input = esp32Serial.readStringUntil('\n');
        input.trim();
        /*Serial.println(input);*/

        if (input.startsWith("BTN:")) {
        handleButtonFrame(input); // ustawia currentMode = MODE_BUTTONS
        } else if (input.startsWith("POS:")) {
        handlePositionFrame(input); // ustawia currentMode = MODE_POSITION
        }
    }

    // Wybór sterowania
    switch (currentMode) {
        case MODE_BUTTONS: controlWithButtons();
        break;

        case MODE_POSITION: controlWithPosition();
        break;

        default: delay(500);
        break;
    }
}



// =========================================================================
// ---------------------------FUNKCJE DODATKOWE-----------------------------
// =========================================================================

// --------------------------- BUTTON mode ---------------------------

void handleButtonFrame(String input) {
  int btnEnd = input.indexOf(';');
  if (btnEnd == -1) return;

  String btnValue = input.substring(4, btnEnd);
  buttonStates = btnValue.toInt();
  currentMode = MODE_BUTTONS;
  /*printButtonStates(buttonStates);*/
}

void controlWithButtons() {
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
    bool limit_Z_hit = (digitalRead(LIMIT_Z_PIN) == HIGH);

    // Detekcja NACIŚNIĘCIA krańcówki Z
    if (limit_Z_hit && !limit_Z_last) {
        Serial.println("!!! Naciśnięto krańcówkę osi Z!");
        // Zapamiętaj aktualny kierunek ruchu
        if (BTN_Z_FRWD) lastDirZ = 1;
        else if (BTN_Z_BACK) lastDirZ = 2;
        Serial.print("Zablokowany kierunek: ");
        Serial.println(lastDirZ == 1 ? "FRWD" : "BACK");
    }
    
    // Detekcja ZWOLNIENIA krańcówki
    if (!limit_Z_hit && limit_Z_last) {
        Serial.println("Zwolniono krańcówkę osi Z - ruch odblokowany");
        lastDirZ = 0;
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

    // Oś Z - blokuj tylko kierunek, w którym naciśnięto krańcówkę
    if (BTN_Z_FRWD && !(limit_Z_hit && lastDirZ == 1)) {
        digitalWrite(DIR_Z, LOW);
        stepPulse(STEP_Z);
    }
    if (BTN_Z_BACK && !(limit_Z_hit && lastDirZ == 2)) {
        digitalWrite(DIR_Z, HIGH);
        stepPulse(STEP_Z);
    }

    delayMicroseconds(STEP_DELAY_MICROS);
}

void stepPulse(int stepPin) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(stepPin, LOW);
}

// --------------------------- POSITION mode ---------------------------
void handlePositionFrame(String input) {
    int axisIndex = 0;
    int start = 4; // pomijamy "POS:"
    while (start < input.length()) {
        int colon1 = input.indexOf(':', start);
        int colon2 = input.indexOf(':', colon1 + 1);
        int semicolon = input.indexOf(';', colon2 + 1);

        if (colon1 == -1 || colon2 == -1 || semicolon == -1) break;

        char axis = input.charAt(start);
        float current = input.substring(colon1 + 1, colon2).toFloat();
        float target = input.substring(colon2 + 1, semicolon).toFloat();

        switch (axis) {
        case 'E': axisIndex = 0; break;
        case 'X': axisIndex = 1; break;
        case 'Y': axisIndex = 2; break;
        case 'Z': axisIndex = 3; break;
        default: continue;
        }

        currentAngles[axisIndex] = current;
        targetAngles[axisIndex] = target;

        start = semicolon + 1;
        currentMode = MODE_POSITION;
    }

    lastPositionUpdate = millis();

    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 5000) {
        for (int i = 0; i < 4; i++) {
            Serial.print("Oś ");
            Serial.print(i);
            Serial.print(": aktualna ");
            Serial.print(currentAngles[i], 2);
            Serial.print("°, cel ");
            Serial.print(targetAngles[i], 2);
            Serial.println("°");
        }
        lastDebug = millis();
    }
}

void controlWithPosition() {

    if (millis() - lastPositionUpdate > POSITION_TIMEOUT) {
        Serial.println("⚠️ TIMEOUT: Brak ramek z ESP32! Zatrzymano sterowanie pozycyjne.");
        currentMode = MODE_NONE;
        return;
    }

    const float tolerance = 0.5; // Dopuszczalny błąd [°]
    const float RAMP_DISTANCE = 30.0; // Odległość zwalniania [°]
    const unsigned int MAX_SPEED = 300; // Najszybszy krok [µs]
    const unsigned int MIN_SPEED = 1000; // Najwolniejszy krok [µs]
    
    unsigned long now = micros();

    for (int i = 0; i < 4; i++) {
        float error = targetAngles[i] - currentAngles[i];

        // Czy dotarliśmy do celu?
        if (abs(error) < tolerance) {
            continue;
        }

        // Wybór pinów
        int dirPin, stepPin, limitPin;
        switch (i) {
            case 0: dirPin = DIR_E; stepPin = STEP_E; limitPin = LIMIT_E_PIN; break;
            case 1: dirPin = DIR_X; stepPin = STEP_X; limitPin = LIMIT_X_PIN; break;
            case 2: dirPin = DIR_Y; stepPin = STEP_Y; limitPin = LIMIT_Y_PIN; break;
            case 3: dirPin = DIR_Z; stepPin = STEP_Z; limitPin = LIMIT_Z_PIN; break;
        }

        // Sprawdź krańcówkę
        bool limitHit = (digitalRead(limitPin) == HIGH);
        bool movingToLimit = (error < 0); // Zakładam, że limit = pozycja 0
        
        if (limitHit && movingToLimit) {
            Serial.print("⚠️ Oś "); 
            Serial.print(i);
            Serial.println("zablokowana przez krańcówkę");
            continue;
        }

        // Profil trapezowy - zwalnianie przy zbliżaniu się do celu
        float absError = abs(error);
        unsigned int stepDelay;
        
        if (absError > RAMP_DISTANCE) {
            // Pełna prędkość - daleko od celu
            stepDelay = MAX_SPEED;
        } else {
            // Zwalniaj proporcjonalnie do odległości
            float speedRatio = absError / RAMP_DISTANCE; // 0.0 - 1.0
            stepDelay = MIN_SPEED - (MIN_SPEED - MAX_SPEED) * speedRatio;
        }

        // Wykonaj krok co odpowiedni interwał
        if (now - lastStepTime[i] >= stepDelay) {
            // Ustaw kierunek
            digitalWrite(dirPin, error > 0 ? LOW : HIGH);
            
            stepPulse(stepPin);
            lastStepTime[i] = now;
        }
    }
}

// --------------------------- INNE ---------------------------

const char* buttonLabels[] = {"X+", "X-", "Y+", "Y-", "Z+", "Z-", "E+", "E-"};
void printButtonStates(byte buttonStates) {
    for (int i = 0; i < 8; ++i) {
        if (i > 0) {
            Serial.print(" | ");
        }
        Serial.print(buttonLabels[i]);
        Serial.print(":");

        if (buttonStates & (1 << i)) {
            Serial.print("ON");
        } else {
            Serial.print("off");
        }
    }
    Serial.println();
}