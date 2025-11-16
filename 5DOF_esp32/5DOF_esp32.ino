#include <Arduino.h>
#include <Wire.h>

// ==================== Tryb symulacji ====================
#define SIMULATION_MODE true  // false = prawdziwe enkodery

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

const uint8_t ENCODER_CHANNEL[] = {4, 5, 6, 7, 3}; // [E, Z, Y, A, X]
const float ENCODER_LEVER[] = {2.0, 3.6, 4.5, 4.5, 4.0}; // Przełożenia
int16_t ENCODER_ZPOS[] = {0, 0, 0, 0, 0}; // Offsety zerowe
int16_t rotationCount[] = {0, 0, 0, 0, 0}; // Liczniki obrotów
uint16_t lastRawAngle[] = {0, 0, 0, 0, 0}; // Poprzednie odczyty
const float angleConst = 360.0 / 4096.0; // Przelicznik raw->stopnie

// ===================== Zmienne sterowania pozycyjnego ======================
float currentAngles[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // [E, Z, Y, A, X] - aktualne kąty
float targetAngles[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // [E, Z, Y, A, X] - docelowe kąty
int32_t targetRawAngles[5] = {0, 0, 0, 0, 0}; // Docelowe raw angle z uwzględnieniem obrotów
unsigned long lastStepTime[5] = {0, 0, 0, 0, 0};

// ===================== Synchronizacja Y i A ======================
const float YA_SYNC_TOLERANCE = 1.0; // Maksymalna różnica między Y i A [°]
unsigned long lastYASync = 0;
const unsigned long YA_SYNC_CHECK_INTERVAL = 500; // Sprawdzanie co 500ms

// ===================== Parametry sterowania ======================
const float ANGLE_TOLERANCE = 0.5; // Tolerancja osiągnięcia celu [°]
const unsigned int STEP_DELAY = 1000; // Opóźnienie między krokami [µs]
const unsigned long ENCODER_READ_INTERVAL = 50;
const unsigned long PYTHON_SEND_INTERVAL = 500;

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
    
    // Odczyt pozycji zerowych enkoderów
    Serial.println("Kalibracja enkoderów...");
    for(int i = 0; i < 5; i++) {
        ENCODER_ZPOS[i] = getEncoderRawAngle(ENCODER_CHANNEL[i]);
        if(ENCODER_ZPOS[i] == 0xFFFF) {
            Serial.printf("❌ BŁĄD: Enkoder %d (kanał %d) nie odpowiada!\n", i, ENCODER_CHANNEL[i]);
        } else {
            const char* axisNames[] = {"E", "Z", "Y", "A", "X"};
            Serial.printf("✓ Enkoder %s: zero = %d\n", axisNames[i], ENCODER_ZPOS[i]);
        }
        lastRawAngle[i] = ENCODER_ZPOS[i];
    }
    
    Serial.println("✓ Inicjalizacja zakończona. Oczekiwanie na ramki z Pythona...");
}

// =========================================================================
// --------------------------------- LOOP ----------------------------------
// =========================================================================

void loop() {
    unsigned long now = millis();
    
    // 1. Odczyt komend z Pythona przez Serial
    readSerialCommands();
    
    // 2. Odczyt enkoderów i aktualizacja pozycji
    if (now - lastEncoderRead >= ENCODER_READ_INTERVAL) {
        lastEncoderRead = now;
        readEncoders();
    }
    
    // 2.5. Sprawdzenie synchronizacji Y i A
    if (now - lastYASync >= YA_SYNC_CHECK_INTERVAL) {
        lastYASync = now;
        checkYASync();
    }
    
    // 3. Sterowanie silnikami do osiągnięcia targetRawAngles
    controlMotorsToTarget();
    
    // 4. Wysyłanie aktualnych pozycji do Pythona
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
        float angle = axisData.substring(1).toFloat();
        
        switch (axis) {
            case 'X': case 'x':
                targetAngles[4] = angle; // Indeks 4 = X
                calculateTargetRaw(4, angle);
                validCommand = true;
                break;
            case 'Y': case 'y': // Y kontroluje ramię A (L2)
                targetAngles[3] = angle;
                targetAngles[2] = angle; // Y i A synchronicznie
                calculateTargetRaw(2, angle);
                calculateTargetRaw(3, angle);
                validCommand = true;
                break;
            case 'Z': case 'z': // Z kontroluje ramię Z (L3)
                targetAngles[1] = angle;
                calculateTargetRaw(1, angle);
                validCommand = true;
                break;
            case 'E': case 'e': // E kontroluje nadgarstek
                targetAngles[0] = angle;
                calculateTargetRaw(0, angle);
                validCommand = true;
                break;
        }
    }
    
    if (validCommand) {
        Serial.println("✓ Otrzymano nowe kąty docelowe:");
        Serial.printf("   E=%.2f° Z=%.2f° Y=%.2f° X=%.2f°\n", 
                      targetAngles[0], targetAngles[1], targetAngles[3], targetAngles[4]);
    }
}

void calculateTargetRaw(int axisIndex, float targetArmAngle) {
    // Przeliczenie kąta ramienia na raw angle enkodera (z uwzględnieniem przełożenia)
    float totalEncoderAngle = targetArmAngle * ENCODER_LEVER[axisIndex];
    
    // Rozbicie na pełne obroty i kąt w aktualnym obrocie
    int32_t fullRotations = (int32_t)(totalEncoderAngle / 360.0);
    float remainingAngle = fmod(totalEncoderAngle, 360.0);
    if (remainingAngle < 0) remainingAngle += 360.0;
    
    // Przeliczenie na raw angle (0-4095)
    int32_t rawInRotation = (int32_t)(remainingAngle / angleConst);
    
    // Dodanie offsetu zerowego
    rawInRotation = (rawInRotation + ENCODER_ZPOS[axisIndex]) % 4096;
    
    // Całkowity raw angle (z obrotami)
    targetRawAngles[axisIndex] = (fullRotations * 4096) + rawInRotation;
}

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
                float velocity = constrain(error * 2.0, -20.0, 20.0);  // max 5°/s
                currentAngles[i] += velocity * dt;
                
                // Aktualizacja rotation count
                rotationCount[i] = (int)(currentAngles[i] / 360.0);
                
                // Przeliczenie currentAngles na lastRawAngle
                float totalEncoderAngle = currentAngles[i] * ENCODER_LEVER[i];
                float remainingAngle = fmod(totalEncoderAngle, 360.0);
                if (remainingAngle < 0) remainingAngle += 360.0;
                
                // Konwersja na raw angle (0-4095)
                int16_t rawAngleAdjusted = (int16_t)(remainingAngle / angleConst);
                lastRawAngle[i] = (rawAngleAdjusted + ENCODER_ZPOS[i]) % 4096;
            }
        }
    #else
        // PRAWDZIWE ENKODERY
        for (int i = 0; i < 5; i++) {
            uint16_t rawAngle = getEncoderRawAngle(ENCODER_CHANNEL[i]);
            
            if (rawAngle == 0xFFFF) { continue; }
            
            int16_t rawAngleAdjusted = rawAngle - ENCODER_ZPOS[i];
            rawAngleAdjusted = ((rawAngleAdjusted % 4096) + 4096) % 4096;
            
            updateRotationCount(i, rawAngleAdjusted);
            
            currentAngles[i] = getArmAngle(rawAngleAdjusted, rotationCount[i], ENCODER_LEVER[i]);
        }
    #endif
}

void controlMotorsToTarget() {
    unsigned long now = micros();
    
    for (int i = 0; i < 5; i++) {
        // Obliczenie aktualnego raw angle (z obrotami)
        int32_t currentRaw = lastRawAngle[i] + (rotationCount[i] * 4096);
        
        // Błąd pozycji
        int32_t error = targetRawAngles[i] - currentRaw;
        
        // Sprawdzenie czy jesteśmy w tolerancji
        float errorDegrees = (error / 4096.0) * 360.0 / ENCODER_LEVER[i];
        if (abs(errorDegrees) < ANGLE_TOLERANCE) {
            continue; // Cel osiągnięty
        }
        
        // Wybór pinów dla danej osi
        int dirPin, stepPin, limitPin;
        
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
            case 3: // A
                dirPin = DIR_A; 
                stepPin = STEP_A; 
                limitPin = LIMIT_Y_PIN; // Wspólna krańcówka z Y
                break;
            case 4: // X
                dirPin = DIR_X; 
                stepPin = STEP_X; 
                limitPin = LIMIT_X_PIN;
                break;
        }
        
        // Sprawdzenie krańcówki
        bool limitHit = (digitalRead(limitPin) == LOW); // Aktywna LOW
        bool movingBack = (error < 0);
        
        if (limitHit && movingBack) {
            continue; // Blokada ruchu w kierunku krańcówki
        }
        
        // Wykonanie kroku z określonym opóźnieniem
        if (now - lastStepTime[i] >= STEP_DELAY) {
            // Kierunek: error > 0 = do przodu, error < 0 = do tyłu
            digitalWrite(dirPin, (error > 0) ? HIGH : LOW);
            
            // Impuls step
            stepPulse(stepPin);
            
            lastStepTime[i] = now;
        }
    }
}

void sendPositionToPython() {
    // Format: <E12.34,Z45.67,Y78.90,X0.00>
    String frame = "<";
    frame += "E" + String(currentAngles[0], 2) + ",";
    frame += "Z" + String(currentAngles[1], 2) + ",";
    frame += "Y" + String(currentAngles[3], 2) + ","; // Wysyłamy kąt A jako Y
    frame += "X" + String(currentAngles[4], 2);
    frame += ">";
    
    Serial.println(frame);
}

void checkYASync() {
    // Y i A działają na tę samą zębatkę, ale w przeciwnych kierunkach
    // Sprawdzamy czy różnica między nimi nie jest zbyt duża
    
    float angleY = currentAngles[2]; // Indeks 2 = Y
    float angleA = currentAngles[3]; // Indeks 3 = A
    
    // Obliczamy różnicę - powinny być niemal identyczne
    // (minus oznacza odwrotny kierunek, ale wartości bezwzględne powinny się zgadzać)
    float difference = abs(angleY - angleA);
    
    if (difference > YA_SYNC_TOLERANCE) {
        Serial.println("⚠️ DESYNCHRONIZACJA Y-A!");
        Serial.printf("   Y = %.2f°, A = %.2f° (różnica = %.2f°)\n", 
                      angleY, angleA, difference);
        
        // Korekcja: ustawiamy A do wartości Y (Y jest referencją)
        targetAngles[3] = angleY;
        calculateTargetRaw(3, angleY);
        
        Serial.println("   → Korekcja: ustawiam A na wartość Y");
    }
}

// =========================================================================
// -------------------------- FUNKCJE ENKODERÓW ----------------------------
// =========================================================================

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

float getArmAngle(int16_t rawAngle, int16_t rotations, float lever) {
    float totalEncoderAngle = (rotations * 360.0) + (rawAngle * angleConst);
    return totalEncoderAngle / lever;
}

void updateRotationCount(int axisIndex, int16_t currentRaw) {
    int32_t lastRaw = lastRawAngle[axisIndex];
    
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