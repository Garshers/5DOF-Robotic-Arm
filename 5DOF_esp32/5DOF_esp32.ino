#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>

// ==================== Tryb symulacji ====================
#define SIMULATION_MODE false  // false = prawdziwe enkodery

// ==================== Podstawowe ustawienia systemowe =====================
#define BAUD 115200

// ======================= Piny silników ======================
#define STEP_X 17
#define DIR_X 16
#define STEP_Y 23
#define DIR_Y 19
#define STEP_A 33
#define DIR_A 32
#define STEP_Z 26
#define DIR_Z 25
#define STEP_E 14
#define DIR_E 27

AccelStepper motorX(AccelStepper::DRIVER, STEP_X, DIR_X);
AccelStepper motorY(AccelStepper::DRIVER, STEP_Y, DIR_Y);
AccelStepper motorA(AccelStepper::DRIVER, STEP_Z, DIR_Z);
AccelStepper motorZ(AccelStepper::DRIVER, STEP_Z, DIR_Z);
AccelStepper motorE(AccelStepper::DRIVER, STEP_E, DIR_E);

#define LIMIT_X_PIN 36
#define LIMIT_Y_PIN 39
#define LIMIT_Z_PIN 34
#define LIMIT_E_PIN 35

// ===================== Konfiguracja enkoderów I2C ======================
#define SDA_PIN 21
#define SCL_PIN 22

const int PCA9548A_ADDR = 0x70;
const int AS5600_ADDR = 0x36;
const int AS5600_RAW_ANGLE_HIGH = 0x0C;

const bool ENCODER_INVERT[] = {true, false, true, true, false}; // [E, Z, Y, A, X]
const uint8_t ENCODER_CHANNEL[] = {4, 5, 6, 7, 3}; // [E, Z, Y, A, X]
const float ENCODER_LEVER[] = {2.0, 3.6, 4.5, 4.5, 4.0}; // Przełożenia
uint16_t ENCODER_ZPOS[] = {0, 0, 0, 0, 0}; // Raw angle w pozycji startowej
int16_t rotationCount[] = {0, 0, 0, 0, 0}; // Liczniki obrotów
uint16_t lastRawAngle[] = {0, 0, 0, 0, 0}; // Poprzednie odczyty
const float angleConst = 360.0 / 4096.0; // Przelicznik raw->stopnie

// ===================== Kąty startowe (pozycja początkowa robota) ======================
// Fizyczne kąty stawów w pozycji startowej
const float START_ANGLES[5] = {90.0, 90.0, 135.0, 135.0, 0.0}; // [E, Z, Y, A, X]

// ===================== Zmienne sterowania pozycyjnego ======================
float currentAngles[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // [E, Z, Y, A, X] - aktualne kąty fizyczne
float targetAngles[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // [E, Z, Y, A, X] - docelowe kąty fizyczne
int32_t targetRawAngles[5] = {0, 0, 0, 0, 0}; // Docelowe raw angle z uwzględnieniem obrotów
unsigned long lastStepTime[5] = {0, 0, 0, 0, 0};

// ===================== Odwrócenie kierunku dla wybranych osi ======================
// true = odwrócony kierunek, false = normalny kierunek
const bool AXIS_INVERT[] = {false, true, false, true, false}; // [E, Z, Y, A, X]

// ===================== Synchronizacja Y i A ======================
const float YA_SYNC_TOLERANCE = 1.0; // Maksymalna różnica między Y i A [°]
unsigned long lastYASync = 0;
const unsigned long YA_SYNC_CHECK_INTERVAL = 500; // Sprawdzanie co 500ms

// ===================== Parametry sterowania ======================
const float ANGLE_TOLERANCE = 0.5; // Tolerancja osiągnięcia celu [°]
const unsigned int STEP_DELAY = 1000; // Opóźnienie między krokami [µs]
const unsigned long ENCODER_READ_INTERVAL = 50;
const unsigned long PYTHON_SEND_INTERVAL = 50;

unsigned long lastEncoderRead = 0;
unsigned long lastPythonSend = 0;

// ===================== Zmienne komunikacji Serial ======================
String serialBuffer = "";

// =========================================================================
// --------------------------------- SETUP ---------------------------------
// =========================================================================

void setup() {
    Serial.begin(BAUD);
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000); // I2C Fast Mode
    
    // Konfiguracja pinów silników
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

    // Konfiguracja krańcówek
    pinMode(LIMIT_X_PIN, INPUT_PULLUP);
    pinMode(LIMIT_Y_PIN, INPUT_PULLUP);
    pinMode(LIMIT_Z_PIN, INPUT_PULLUP);
    pinMode(LIMIT_E_PIN, INPUT_PULLUP);

    delay(500);
    
    Serial.println("=== Inicjalizacja ESP32 - Sterowanie enkoderowe ===");
    
    const char* axisNames[] = {"E", "Z", "Y", "A", "X"};
    
    // Wyświetlenie kątów startowych
    Serial.println("Kąty startowe (fizyczne):");
    Serial.printf("  E=%.1f° Z=%.1f° Y=%.1f° A=%.1f° X=%.1f°\n",
                  START_ANGLES[0], START_ANGLES[1], START_ANGLES[2], START_ANGLES[3], START_ANGLES[4]);
    
    // Odczyt pozycji zerowych enkoderów (w pozycji startowej)
    Serial.println("Kalibracja enkoderów w pozycji startowej...");
    for(int i = 0; i < 5; i++) {
        uint16_t rawReading = getEncoderRawAngle(ENCODER_CHANNEL[i]);
        
        if(rawReading == 0xFFFF) {
            Serial.printf("❌ BŁĄD: Enkoder %s (kanał %d) nie odpowiada!\n", 
                          axisNames[i], ENCODER_CHANNEL[i]);
            ENCODER_ZPOS[i] = 0;
        } else {
            // Zapisujemy surowy odczyt jako pozycję zerową (startową)
            ENCODER_ZPOS[i] = rawReading;
            Serial.printf("✓ Enkoder %s: raw=%d (start=%.1f°)\n", 
                          axisNames[i], ENCODER_ZPOS[i], START_ANGLES[i]);
        }
        
        lastRawAngle[i] = ENCODER_ZPOS[i];
        rotationCount[i] = 0; // Startujemy z licznikiem obrotów = 0
        
        // Ustawienie początkowych kątów jako kąty startowe
        currentAngles[i] = START_ANGLES[i];
        targetAngles[i] = START_ANGLES[i];
    }
    
    // Obliczenie początkowych targetRawAngles (pozycja startowa)
    for(int i = 0; i < 5; i++) {
        calculateTargetRaw(i, START_ANGLES[i]);
    }
    
    Serial.println("✓ Inicjalizacja zakończona. Oczekiwanie na ramki z Pythona...");
}

// =========================================================================
// --------------------------------- LOOP ----------------------------------
// =========================================================================

void testMotor(AccelStepper& motor, const char* motorName, 
               long steps, float maxSpeed, float acceleration, 
               bool returnToZero = true, int delayAfter = 1000) {
    
    Serial.printf("=== TEST: Silnik %s ===\n", motorName);
    Serial.printf("Parametry: %ld kroków, %.0f kr/s, %.0f kr/s²\n", 
                  steps, maxSpeed, acceleration);
    
    // Konfiguracja parametrów ruchu
    motor.setMaxSpeed(maxSpeed);
    motor.setAcceleration(acceleration);
    
    long startPos = motor.currentPosition();
    long targetPos = startPos + steps;
    
    // ===== Ruch do celu =====
    Serial.printf(" -> Ruch z pozycji %ld do %ld\n", startPos, targetPos);
    motor.moveTo(targetPos);
    
    unsigned long startTime = millis();
    long lastReportPos = startPos;
    
    while (motor.distanceToGo() != 0) {
        motor.run();
        
        // Raportuj postęp co 10 kroków
        long currentPos = motor.currentPosition();
        if (abs(currentPos - lastReportPos) >= 10) {
            Serial.printf("   Pozycja: %ld / %ld\n", currentPos, targetPos);
            lastReportPos = currentPos;
        }
    }
    
    unsigned long duration = millis() - startTime;
    Serial.printf(" -> Osiągnięto pozycję: %ld (czas: %lu ms)\n", 
                  motor.currentPosition(), duration);
    
    if (delayAfter > 0) {
        Serial.printf(" -> Czekam %d ms...\n", delayAfter);
        delay(delayAfter);
    }
    
    // ===== Opcjonalny powrót =====
    if (returnToZero) {
        Serial.printf(" -> Powrót do pozycji startowej (%ld)\n", startPos);
        motor.moveTo(startPos);
        
        startTime = millis();
        lastReportPos = motor.currentPosition();
        
        while (motor.distanceToGo() != 0) {
            motor.run();
            
            long currentPos = motor.currentPosition();
            if (abs(currentPos - lastReportPos) >= 10) {
                Serial.printf("   Pozycja: %ld / %ld\n", currentPos, startPos);
                lastReportPos = currentPos;
            }
        }
        
        duration = millis() - startTime;
        Serial.printf(" -> Osiągnięto pozycję: %ld (czas: %lu ms)\n", 
                      motor.currentPosition(), duration);
    }
    
    Serial.printf("=== KONIEC TESTU: %s ===\n\n", motorName);
}

void loop() {
    unsigned long now = millis();
    
    // Odczyt komend z Pythona przez Serial
    readSerialCommands();
    
    // Odczyt enkoderów i aktualizacja pozycji
    if (now - lastEncoderRead >= ENCODER_READ_INTERVAL) {
        lastEncoderRead = now;
        readEncoders();
    }
    
    // Sprawdzenie synchronizacji Y i A
    if (now - lastYASync >= YA_SYNC_CHECK_INTERVAL) {
        lastYASync = now;
        //checkYASync();
    }
    
    // TESTOWANIE
    testMotor(motorX, "X", 100, 500, 200, true, 1000);
    
    // Wysyłanie aktualnych pozycji do Pythona
    if (now - lastPythonSend >= PYTHON_SEND_INTERVAL) {
        lastPythonSend = now;
        sendPositionToPython();
    }
}

// =========================================================================
// -------------------------- FUNKCJE GŁÓWNE -------------------------------
// =========================================================================

void readSerialCommands() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        if (c == '\n' || c == '\r') {
            if (serialBuffer.length() > 0) {
                parsePythonCommand(serialBuffer);
                serialBuffer = "";
            }
        } else {
            serialBuffer += c;
        }
    }
}

void parsePythonCommand(String cmd){
    // Format: X12.34,Y45.67,Z78.90,E12.34
    // Kąty od Pythona są RELATYWNE względem pozycji startowej (0 = pozycja startowa)
    cmd.trim();
    
    int startIdx = 0;
    bool validCommand = false;
    
    while (startIdx < cmd.length()) {
        int commaIdx = cmd.indexOf(',', startIdx);
        String axisData;
        
        if (commaIdx == -1) {
            axisData = cmd.substring(startIdx);
            startIdx = cmd.length();
        } else {
            axisData = cmd.substring(startIdx, commaIdx);
            startIdx = commaIdx + 1;
        }
        
        if (axisData.length() < 2) continue;
        
        char axis = axisData.charAt(0);
        float relativeAngle = axisData.substring(1).toFloat();
        
        // Konwersja: kąt relatywny -> kąt fizyczny (absolutny)
        switch (axis) {
            case 'X': case 'x':
                targetAngles[4] = relativeAngle + START_ANGLES[4]; // Indeks 4 = X
                calculateTargetRaw(4, targetAngles[4]);
                validCommand = true;
                break;
            case 'Y': case 'y': // Y kontroluje ramię A (L2)
                targetAngles[3] = relativeAngle + START_ANGLES[3];
                targetAngles[2] = relativeAngle + START_ANGLES[2]; // Y i A synchronicznie
                calculateTargetRaw(2, targetAngles[2]);
                calculateTargetRaw(3, targetAngles[3]);
                validCommand = true;
                break;
            case 'Z': case 'z': // Z kontroluje ramię Z (L3)
                targetAngles[1] = relativeAngle + START_ANGLES[1];
                calculateTargetRaw(1, targetAngles[1]);
                validCommand = true;
                break;
            case 'E': case 'e': // E kontroluje nadgarstek
                targetAngles[0] = relativeAngle + START_ANGLES[0];
                calculateTargetRaw(0, targetAngles[0]);
                validCommand = true;
                break;
        }
    }
    
    if (validCommand) {
        Serial.println("✓ Otrzymano nowe kąty docelowe (fizyczne):");
        Serial.printf("   E=%.2f° Z=%.2f° Y=%.2f° X=%.2f°\n", 
                      targetAngles[0], targetAngles[1], targetAngles[3], targetAngles[4]);
    }
}

void calculateTargetRaw(int axisIndex, float targetPhysicalAngle) {
    float angleDifference = targetPhysicalAngle - START_ANGLES[axisIndex];
    
    if (ENCODER_INVERT[axisIndex]) {
        angleDifference = -angleDifference;
    }

    float encoderAngleDifference = angleDifference * ENCODER_LEVER[axisIndex];
    
    // Rozbicie na pełne obroty i kąt w aktualnym obrocie
    int32_t fullRotations = (int32_t)floor(encoderAngleDifference / 360.0);
    float remainingAngle = encoderAngleDifference - (fullRotations * 360.0);
    
    // Przeliczenie na raw angle (0-4095)
    int32_t rawInRotation = (int32_t)(remainingAngle / angleConst);
    
    // Dodanie offsetu zerowego
    rawInRotation = (rawInRotation + ENCODER_ZPOS[axisIndex]) % 4096;
    
    // Całkowity raw angle (z obrotami)
    targetRawAngles[axisIndex] = (fullRotations * 4096) + rawInRotation;
}

void sendPositionToPython() {
    // Format: <E12.34,Z45.67,Y78.90,X0.00>
    // (Python otrzymuje <90,90,135,0> gdy robot jest w pozycji startowej)
    String frame = "<";
    frame += "E" + String(currentAngles[0], 2) + ",";
    frame += "Z" + String(currentAngles[1], 2) + ",";
    frame += "Y" + String(currentAngles[3], 2) + ","; // Wysyłamy kąt A jako Y
    frame += "X" + String(currentAngles[4], 2);
    frame += ">";
    
    Serial.println(frame);
}

// =========================================================================
// -------------------------- FUNKCJE ENKODERÓW ----------------------------
// =========================================================================

void readEncoders() {
    #if SIMULATION_MODE
        // SYMULACJA
        static uint32_t lastUpdate = 0;
        uint32_t now = millis();
        
        if (now - lastUpdate > 50) {  // Aktualizacja co 50ms
            float dt = (now - lastUpdate) / 1000.0;
            lastUpdate = now;
            
            for (int i = 0; i < 5; i++) {
                // Prosta symulacja: powolne podążanie za targetAngles
                float error = targetAngles[i] - currentAngles[i];
                float velocity = constrain(error * 2.0, -20.0, 20.0);  // max 20°/s
                currentAngles[i] += velocity * dt;
                
                // Przeliczenie na raw angle z obrotami
                float angleDifference = currentAngles[i] - START_ANGLES[i];
                float encoderAngleDifference = angleDifference * ENCODER_LEVER[i];
                
                rotationCount[i] = (int32_t)(encoderAngleDifference / 360.0);
                float remainingAngle = fmod(encoderAngleDifference, 360.0);
                if (remainingAngle < 0) remainingAngle += 360.0;
                
                // Konwersja na raw angle (0-4095)
                uint16_t rawAngleAdjusted = (uint16_t)(remainingAngle / angleConst);
                lastRawAngle[i] = (rawAngleAdjusted + ENCODER_ZPOS[i]) % 4096;
            }
        }
    #else
        // PRAWDZIWE ENKODERY
        for (int i = 0; i < 5; i++) {
            uint16_t rawAngle = getEncoderRawAngle(ENCODER_CHANNEL[i]);
            
            if (rawAngle == 0xFFFF) { continue; }
            
            // Aktualizacja licznika obrotów (wykrycie przejścia przez zero)
            updateRotationCount(i, rawAngle);
            
            // Obliczenie całkowitego raw angle (z obrotami)
            int32_t totalRawAngle = rawAngle + (rotationCount[i] * 4096);
            
            // Obliczenie różnicy od pozycji startowej
            int32_t rawDifference = totalRawAngle - ENCODER_ZPOS[i];
            
            // Przeliczenie różnicy raw angle na kąt enkodera (w stopniach)
            float encoderAngleDifference = (rawDifference / 4096.0) * 360.0;
            
            // Uwzględnienie kierunku inkrementacji przed przełożeniem
            if (ENCODER_INVERT[i]) {
                encoderAngleDifference = -encoderAngleDifference;
            }

            // Przeliczenie na kąt fizyczny ramienia (przez przełożenie)
            float armAngleDifference = encoderAngleDifference / ENCODER_LEVER[i];
            
            // Kąt fizyczny = kąt startowy + różnica
            currentAngles[i] = START_ANGLES[i] + armAngleDifference;
        }
    #endif
}

bool selectI2CChannel(uint8_t channel) {
    Wire.beginTransmission(PCA9548A_ADDR);
    Wire.write(1 << channel);
    return (Wire.endTransmission() == 0);
}

bool isAS5600Available() {
    Wire.beginTransmission(AS5600_ADDR);
    return (Wire.endTransmission(true) == 0);
}

bool readAS5600Raw(uint16_t &angle) {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_RAW_ANGLE_HIGH);
    if (Wire.endTransmission(false) != 0) return false;

    Wire.requestFrom(AS5600_ADDR, 2);
    if (Wire.available() != 2) return false;

    uint8_t high = Wire.read();
    uint8_t low = Wire.read();
    angle = ((high << 8) | low) & 0x0FFF;
    return true;
}

uint16_t getEncoderRawAngle(uint8_t channel) {
    if (!selectI2CChannel(channel)) {
        return 0xFFFF;
    }

    if (!isAS5600Available()) {
        return 0xFFFF;
    }

    uint16_t angle;
    if (!readAS5600Raw(angle)) {
        return 0xFFFF;
    }

    return angle;
}

void updateRotationCount(int axisIndex, uint16_t currentRaw) {
    uint16_t lastRaw = lastRawAngle[axisIndex];
    
    // Wykrycie przejścia 4095->0 (obrót w przód)
    if (lastRaw > 3000 && currentRaw < 1000) {
        rotationCount[axisIndex]++;
    }
    // Wykrycie przejścia 0->4095 (obrót w tył)
    else if (lastRaw < 1000 && currentRaw > 3000) {
        rotationCount[axisIndex]--;
    }
    
    lastRawAngle[axisIndex] = currentRaw;
}

// =========================================================================
// --------------------------- FUNKCJE POMOCNICZE --------------------------
// =========================================================================

void stepPulse(int stepPin) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(stepPin, LOW);
}