#include <Arduino.h>
#include <AccelStepper.h>

// ==================== Podstawowe ustawienia systemowe =====================
#define BAUD 115200 // Prędkość komunikacji ESP32 -> Komputer

// =================== Zmienne dotyczące komunikacji UART ===================
// ESP32 ma wbudowane porty UART - używamy UART2
#define UART_RX_PIN 16 // RX2
#define UART_TX_PIN 17 // TX2
#define UART_BAUD 57600 // Szybkość transmisji
byte buttonStates = 0; // Zmienna przechowująca stany przycisków
HardwareSerial esp32Serial(2); // UART2 na ESP32
unsigned long lastInputTime = 0;

// ======================= Zmienne sterujące silnikami ======================
#define STEP_X 23
#define DIR_X 22
#define STEP_Y 21
#define DIR_Y 19
#define STEP_A 33
#define DIR_A 32
#define STEP_Z 26
#define DIR_Z 25
#define STEP_E 14 
#define DIR_E 27

//25+26; 32+33; 21+19; 23+22; 27+14

#define LIMIT_X_PIN 0
#define LIMIT_Y_PIN 0
#define LIMIT_Z_PIN 0
#define LIMIT_E_PIN 0

// Definicje obiektów AccelStepper
AccelStepper stepperX(AccelStepper::DRIVER, STEP_X, DIR_X);
AccelStepper stepperY(AccelStepper::DRIVER, STEP_Y, DIR_Y);
AccelStepper stepperA(AccelStepper::DRIVER, STEP_A, DIR_A);
AccelStepper stepperZ(AccelStepper::DRIVER, STEP_Z, DIR_Z);
AccelStepper stepperE(AccelStepper::DRIVER, STEP_E, DIR_E);

enum ControlMode { MODE_NONE, MODE_BUTTONS, MODE_POSITION };
ControlMode currentMode = MODE_NONE;

bool limit_Z_last = false; // Zmienna do wykrywania zbocza na krańcówce Z
int lastDirZ = 0; // Ostatni kierunek ruchu Z (1 = FRWD/LOW, 2 = BACK/HIGH, 0 = brak ruchu)
bool limit_Z_blocked = false; // Flaga blokady kierunku

// ======================== Sterowanie pozycyjne ========================

const char axisLabels[5] = {'E', 'Z', 'Y', 'A', 'X'};
float currentAngles[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float targetAngles[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
unsigned long lastStepTime[5] = {0, 0, 0, 0, 0};
unsigned long lastPositionUpdate = 0; // Timeout dla trybu pozycyjnego
const unsigned long POSITION_TIMEOUT = 3000; // 3 sekundy

// Synchronizacja Y i A
const float SYNC_TOLERANCE = 0.5; // Próg różnicy w stopniach

void setup() {
    pinMode(STEP_X, OUTPUT);
    pinMode(DIR_X, OUTPUT);
    pinMode(STEP_Y, OUTPUT);
    pinMode(DIR_Y, OUTPUT);
    pinMode(STEP_A, OUTPUT);
    pinMode(DIR_A, OUTPUT);
    pinMode(STEP_Z, OUTPUT);
    pinMode(DIR_Z, OUTPUT);
    pinMode(STEP_E, OUTPUT);
    pinMode(DIR_E, OUTPUT);

    pinMode(LIMIT_X_PIN, INPUT_PULLUP);
    pinMode(LIMIT_Y_PIN, INPUT_PULLUP);
    pinMode(LIMIT_Z_PIN, INPUT_PULLUP);
    pinMode(LIMIT_E_PIN, INPUT_PULLUP);

    // Konfiguracja silników
    stepperE.setMaxSpeed(2000);    // kroki/s
    stepperE.setAcceleration(1000); // kroki/s²

    stepperX.setMaxSpeed(2000);
    stepperX.setAcceleration(1000);
    
    stepperY.setMaxSpeed(2000);
    stepperY.setAcceleration(1000);
    
    stepperA.setMaxSpeed(2000);
    stepperA.setAcceleration(1000);
    stepperA.setPinsInverted(true, false, false); // Oś A odwrócona fizycznie

    stepperZ.setMaxSpeed(2000);
    stepperZ.setAcceleration(1000);

    Serial.begin(BAUD);
    delay(500); // Krótkie opóźnienie dla stabilności

    Serial.println("=== ESP32 - Ustawienia komunikacji UART ===");
    
    // Inicjalizacja UART2 z określonymi pinami
    esp32Serial.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    
    Serial.print("UART2 RX: GPIO");
    Serial.print(UART_RX_PIN);
    Serial.print(", TX: GPIO");
    Serial.print(UART_TX_PIN);
    Serial.print(", Baud: ");
    Serial.print(UART_BAUD);
    Serial.println(". Czekam na dane...");
}


void loop() {
    if (esp32Serial.available()) {
        String input = esp32Serial.readStringUntil('\n');
        input.trim();
        
        if (millis() - lastInputTime >= 500) {
            Serial.println(input);
            lastInputTime = millis();
        }


        if (input.startsWith("BTN:")) {
            handleButtonFrame(input); // ustawia currentMode = MODE_BUTTONS
        } else if (input.startsWith("POS:")) {
            handlePositionFrame(input); // ustawia currentMode = MODE_POSITION
        }
    }

    // Wybór sterowania
    switch (currentMode) {
        case MODE_BUTTONS: 
            controlWithButtons();
            break;

        case MODE_POSITION: 
            controlWithPosition();
            break;

        default: 
            delay(5);
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
    //printButtonStates(buttonStates);
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
    bool limit_X_hit = (digitalRead(LIMIT_X_PIN) == HIGH);
    bool limit_Y_hit = (digitalRead(LIMIT_Y_PIN) == HIGH);
    bool limit_Z_hit = (digitalRead(LIMIT_Z_PIN) == HIGH);
    bool limit_E_hit = (digitalRead(LIMIT_E_PIN) == HIGH);

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

    const long CONTINUOUS_SPEED = 100; // Prędkość ciągłego ruchu [kroki/s]

    // Oś E
    if (BTN_E_FRWD) {
        stepperE.setSpeed(CONTINUOUS_SPEED);
        stepperE.runSpeed();
    } else if (BTN_E_BACK) {
        stepperE.setSpeed(-CONTINUOUS_SPEED);
        stepperE.runSpeed();
    } else {
        stepperE.setSpeed(0);
    }

    // Oś X
    if (BTN_X_FRWD) {
        stepperX.setSpeed(CONTINUOUS_SPEED);
        stepperX.runSpeed();
    } else if (BTN_X_BACK) {
        stepperX.setSpeed(-CONTINUOUS_SPEED);
        stepperX.runSpeed();
    } else {
        stepperX.setSpeed(0);
    }

    // Osie Y i A - poruszają się razem, A jest odwrócona sprzętowo
    if (BTN_Y_FRWD) {
        stepperY.setSpeed(CONTINUOUS_SPEED);
        stepperY.runSpeed();
        stepperA.setSpeed(CONTINUOUS_SPEED);
        stepperA.runSpeed();
    } else if (BTN_Y_BACK) {
        stepperY.setSpeed(-CONTINUOUS_SPEED);
        stepperY.runSpeed();
        stepperA.setSpeed(-CONTINUOUS_SPEED);
        stepperA.runSpeed();
    } else {
        stepperY.setSpeed(0);
        stepperA.setSpeed(0);
    }

    // Oś Z - blokuj tylko kierunek, w którym naciśnięto krańcówkę
    if (BTN_Z_FRWD) {
        stepperZ.setSpeed(CONTINUOUS_SPEED);
        stepperZ.runSpeed();
    } else if (BTN_Z_BACK) {
        stepperZ.setSpeed(-CONTINUOUS_SPEED);
        stepperZ.runSpeed();
    } else {
        stepperZ.setSpeed(0);
    }
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
            case 'Z': axisIndex = 1; break;
            case 'Y': 
                axisIndex = 2;
                // Synchronizuj cel dla osi A z osią Y
                currentAngles[3] = current;
                targetAngles[3] = target;
                break;
            case 'A': axisIndex = 3; break;
            case 'X': axisIndex = 4; break;
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
        for (int i = 0; i < 5; i++) {
            Serial.print("Oś ");
            Serial.print(axisLabels[i]);
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
    const unsigned int STEP_DELAY = 1000; // Stała prędkość [µs]
    
    unsigned long now = micros();

    // Sprawdź synchronizację Y i A
    float diffYA = currentAngles[2] - currentAngles[3];
    bool needsSync = abs(diffYA) > SYNC_TOLERANCE;
    
    if (needsSync && millis() % 5000 < 10) {
        Serial.print("⚠️ Synchronizacja Y-A: różnica = ");
        Serial.print(diffYA, 2);
        Serial.println("°");
    }

    for (int i = 0; i < 5; i++) {
        float error = targetAngles[i] - currentAngles[i];

        // Czy dotarliśmy do celu?
        if (abs(error) < tolerance) {
            continue;
        }

        // Wybór pinów
        int dirPin, stepPin, limitPin;
        bool invertDirection = false;
        
        switch (i) {
            case 0: // E
                dirPin = DIR_E; 
                stepPin = STEP_E; 
                limitPin = LIMIT_E_PIN; 
                break;
            case 1: // Z
                dirPin = DIR_Z; 
                stepPin = STEP_Z; 
                limitPin = LIMIT_Z_PIN; 
                break;
            case 2: // Y
                dirPin = DIR_Y; 
                stepPin = STEP_Y; 
                limitPin = LIMIT_Y_PIN;
                break;
            case 3: // A (odwrócona sprzętowo przez setPinsInverted)
                dirPin = DIR_A; 
                stepPin = STEP_A; 
                limitPin = LIMIT_Y_PIN; // używa tej samej krańcówki co Y
                
                // Pętla zwrotna: jeśli A jest w tyle za Y, przyspiesz
                if (needsSync && diffYA > SYNC_TOLERANCE) {
                    // Y jest przed A, A musi dogonić
                    error = targetAngles[2] - currentAngles[3]; // cel Y - pozycja A
                }
                break;
            case 4: // X
                dirPin = DIR_X; 
                stepPin = STEP_X; 
                limitPin = LIMIT_X_PIN; 
                break;
        }

        // Sprawdź krańcówkę
        bool limitHit = (digitalRead(limitPin) == HIGH);
        bool movingBack = (error < 0);
        
        // Zablokuj ruch w kierunku BACK gdy krańcówka naciśnięta
        if (limitHit && movingBack) {
            continue;
        }

        // Wykonaj krok co STEP_DELAY
        if (now - lastStepTime[i] >= STEP_DELAY) {
            // Kierunek: error > 0 = FORWARD (HIGH), error < 0 = BACK (LOW)
            bool direction = (error > 0) ? HIGH : LOW;
            
            digitalWrite(dirPin, direction);
            stepPulse(stepPin);
            lastStepTime[i] = now;
        }
    }
}

// --------------------------- INNE ---------------------------

void stepPulse(int stepPin) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(stepPin, LOW);
}

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