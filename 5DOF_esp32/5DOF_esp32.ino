#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>

// ==================== Tryb symulacji ====================
#define SIMULATION_MODE false

// ==================== Podstawowe ustawienia ====================
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
AccelStepper motorA(AccelStepper::DRIVER, STEP_A, DIR_A);
AccelStepper motorZ(AccelStepper::DRIVER, STEP_Z, DIR_Z);
AccelStepper motorE(AccelStepper::DRIVER, STEP_E, DIR_E);

#define LIMIT_X_PIN 36
#define LIMIT_Y_PIN 39
#define LIMIT_Z_PIN 34
#define LIMIT_E_PIN 35

// ===================== Konfiguracja I2C ======================
#define SDA_PIN 21
#define SCL_PIN 22

const int PCA9548A_ADDR = 0x70;
const int AS5600_ADDR = 0x36;
const int AS5600_RAW_ANGLE_HIGH = 0x0C;

const bool ENCODER_INVERT[] = {true, false, true, true, false}; // [E, Z, Y, A, X]
const uint8_t ENCODER_CHANNEL[] = {4, 5, 6, 7, 3};
const float ENCODER_LEVER[] = {2.0, 3.6, 4.5, 4.5, 4.0};

// ===================== Zmienne współdzielone (z mutex) ======================
SemaphoreHandle_t xMutex;

// Chronione przez mutex
volatile float currentAngles[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // [E, Z, Y, A, X]
volatile float targetAngles[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
volatile bool newTargetAvailable = false;

// Dane enkoderów (tylko rdzeń 0)
uint16_t ENCODER_ZPOS[] = {0, 0, 0, 0, 0};
int16_t rotationCount[] = {0, 0, 0, 0, 0};
uint16_t lastRawAngle[] = {0, 0, 0, 0, 0};
const float angleConst = 360.0 / 4096.0;

// ===================== Kąty startowe ======================
const float START_ANGLES[5] = {90.0, 90.0, 135.0, 135.0, 0.0}; // [E, Z, Y, A, X]

// ===================== Parametry sterowania ======================
const bool AXIS_INVERT[] = {false, false, false, false, false};
const float ANGLE_TOLERANCE = 0.05;
const unsigned long ENCODER_READ_INTERVAL = 50;
const unsigned long PYTHON_SEND_INTERVAL = 50;

// ================== Parametry silnika krokowego ==================
const float STEPS_PER_REV = 200.0;  // Silnik 1.8 stopnia
const float MICROSTEPS = 1.0;//16.0;      // Ustawienie sterownika
const float STEPS_PER_MOTOR_REV = STEPS_PER_REV * MICROSTEPS; // 3200 kroków/obrót silnika

// Bufory komunikacji (tylko rdzeń 0)
String serialBuffer = "";

// =========================================================================
// ----------------------------- RDZEŃ 0 -----------------------------------
// ---------------------- KOMUNIKACJA I ENKODERY ---------------------------
// =========================================================================

void communicationTask(void *parameter) {
    unsigned long lastEncoderRead = 0;
    unsigned long lastPythonSend = 0;
    
    Serial.println("✓ Rdzeń 0: Task komunikacyjny uruchomiony");
    
    while (true) {
        unsigned long now = millis();
        
        // ===== Odczyt komend z Pythona =====
        readSerialCommands();
        
        // ===== Odczyt enkoderów =====
        if (now - lastEncoderRead >= ENCODER_READ_INTERVAL) {
            lastEncoderRead = now;
            readEncoders();
        }
        
        // ===== Wysyłanie pozycji do Pythona =====
        if (now - lastPythonSend >= PYTHON_SEND_INTERVAL) {
            lastPythonSend = now;
            sendPositionToPython();
        }
        
        vTaskDelay(1); // Oddanie czasu procesorowi (1ms)
    }
}

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

void parsePythonCommand(String cmd) {
    cmd.trim();
    
    int startIdx = 0;
    bool validCommand = false;
    float newTargets[5];
    
    // Kopiuj aktualne cele
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
        memcpy(newTargets, (void*)targetAngles, sizeof(newTargets));
        xSemaphoreGive(xMutex);
    }
    
    // Parsowanie
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
        
        switch (axis) {
            case 'X': case 'x':
                newTargets[4] = relativeAngle;
                validCommand = true;
                break;
            case 'Y': case 'y':
                newTargets[3] = relativeAngle;
                newTargets[2] = relativeAngle;
                validCommand = true;
                break;
            case 'Z': case 'z':
                newTargets[1] = relativeAngle;
                validCommand = true;
                break;
            case 'E': case 'e':
                newTargets[0] = relativeAngle;
                validCommand = true;
                break;
        }
    }
    
    // Zapisz nowe cele (chronione mutexem)
    if (validCommand) {
        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
            memcpy((void*)targetAngles, newTargets, sizeof(newTargets));
            newTargetAvailable = true;
            xSemaphoreGive(xMutex);
        }
        
        Serial.println("✓ Nowe kąty docelowe:");
        Serial.printf("   E=%.2f° Z=%.2f° Y=%.2f° X=%.2f°\n", 
                      newTargets[0], newTargets[1], newTargets[3], newTargets[4]);
    }
}

void sendPositionToPython() {
    float angles[5];
    
    // Bezpieczne skopiowanie danych
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
        memcpy(angles, (void*)currentAngles, sizeof(angles));
        xSemaphoreGive(xMutex);
    }
    
    String frame = "<";
    frame += "E" + String(angles[0], 2) + ",";
    frame += "Z" + String(angles[1], 2) + ",";
    frame += "Y" + String(angles[2], 2) + ",";
    frame += "A" + String(angles[3], 2) + ",";
    frame += "X" + String(angles[4], 2);
    frame += ">";
    
    Serial.println(frame);
}

void readEncoders() {
    #if SIMULATION_MODE
        // SYMULACJA
        static uint32_t lastUpdate = 0;
        uint32_t now = millis();
        
        if (now - lastUpdate > 50) {
            float dt = (now - lastUpdate) / 1000.0;
            lastUpdate = now;
            
            float tempCurrent[5], tempTarget[5];
            
            if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
                memcpy(tempCurrent, (void*)currentAngles, sizeof(tempCurrent));
                memcpy(tempTarget, (void*)targetAngles, sizeof(tempTarget));
                xSemaphoreGive(xMutex);
            }
            
            for (int i = 0; i < 5; i++) {
                float error = tempTarget[i] - tempCurrent[i];
                float velocity = constrain(error * 2.0, -20.0, 20.0);
                tempCurrent[i] += velocity * dt;
            }
            
            if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
                memcpy((void*)currentAngles, tempCurrent, sizeof(tempCurrent));
                xSemaphoreGive(xMutex);
            }
        }
    #else
        // PRAWDZIWE ENKODERY
        float tempAngles[5];
        
        for (int i = 0; i < 5; i++) {
            uint16_t rawAngle = getEncoderRawAngle(ENCODER_CHANNEL[i]);
            
            if (rawAngle == 0xFFFF) { 
                tempAngles[i] = currentAngles[i]; // Zachowaj ostatnią wartość
                continue; 
            }
            
            updateRotationCount(i, rawAngle);
            
            int32_t totalRawAngle = rawAngle + (rotationCount[i] * 4096);
            int32_t rawDifference = totalRawAngle - ENCODER_ZPOS[i];
            float encoderAngleDifference = (rawDifference / 4096.0) * 360.0;
            
            if (ENCODER_INVERT[i]) {
                encoderAngleDifference = -encoderAngleDifference;
            }
            
            float armAngleDifference = encoderAngleDifference / ENCODER_LEVER[i];
            tempAngles[i] = START_ANGLES[i] + armAngleDifference;
        }
        
        // Aktualizacja chronionej zmiennej
        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
            memcpy((void*)currentAngles, tempAngles, sizeof(tempAngles));
            xSemaphoreGive(xMutex);
        }
    #endif
}

// =========================================================================
// ----------------------------- RDZEŃ 1 -----------------------------------
// ------------------------ STEROWANIE SILNIKAMI ---------------------------
// =========================================================================

void motorControlTask(void *parameter) {
    long targetSteps[5] = {0, 0, 0, 0, 0};
    float localTargetAngles[5];
    float localCurrentAngles[5];
    float stepsPerDegree[5];
    
    // Niezależne timery dla każdej osi w celu uniknięcia oscylacji
    unsigned long lastAxisCorrectionTime[5] = {0, 0, 0, 0, 0};
    const unsigned long CORRECTION_INTERVAL = 200; 
    
    // Wskaźniki na obiekty silników
    AccelStepper* motors[5] = {&motorE, &motorZ, &motorY, &motorA, &motorX};

    // Konfiguracja silników
    float baseSpeed = 200;
    float baseAcceleration = 100;
    
    for(int i=0; i<5; i++) {
        motors[i]->setMaxSpeed(baseSpeed * ENCODER_LEVER[i]);
        motors[i]->setAcceleration(baseAcceleration * ENCODER_LEVER[i]);
        stepsPerDegree[i] = (STEPS_PER_MOTOR_REV * ENCODER_LEVER[i]) / 360.0;
    }
    
    Serial.println("✓ Rdzeń 1: Task sterowania z absolutną korekcją (moveTo) uruchomiony");
    
    while (true) {
        unsigned long currentMillis = millis();

        // ===== 1. Pobieranie danych (Setpoints & Feedback) =====
        bool hasNewTarget = false;
        if (xSemaphoreTake(xMutex, 10 / portTICK_PERIOD_MS)) {
            if (newTargetAvailable) {
                memcpy(localTargetAngles, (void*)targetAngles, sizeof(localTargetAngles));
                newTargetAvailable = false;
                hasNewTarget = true;
            }
            // Aktualizacja sprzężenia zwrotnego pozycji
            memcpy(localCurrentAngles, (void*)currentAngles, sizeof(localCurrentAngles));
            xSemaphoreGive(xMutex);
        }
        
        // ===== 2. Sterowanie w pętli zamkniętej (Real-time Closed Loop) =====
        for (int i = 0; i < 5; i++) {
            // Pomiń niezależne obliczenia dla osi A (indeks 3), która jest Slavem
            if (i == 3) continue;

            // Obliczenie uchybu: e(t) = wartość_zadana - wartość_mierzona
            float error = localTargetAngles[i] - localCurrentAngles[i];

            // Weryfikacja strefy nieczułości (deadband)
            if (abs(error) > ANGLE_TOLERANCE) {
                
                long stepsCorrection = (long)(error * stepsPerDegree[i]);

                if (AXIS_INVERT[i]) {
                    stepsCorrection = -stepsCorrection;
                }

                // Aplikacja ruchu dla silnika bieżącego (w tym Mastera Y)
                motors[i]->moveTo(motors[i]->currentPosition() + stepsCorrection);
                
                // --- LOGIKA MASTER-SLAVE DLA OSI A (Z AUTOKOREKCJĄ) ---
                if (i == 2) {
                    // Weryfikacja: Zamiast kopiować ruch Y, obliczamy uchyb bezpośrednio dla A.
                    // Cel jest ten sam (localTargetAngles[2]), ale pozycja startowa A może być inna.
                    float errorA = localTargetAngles[2] - localCurrentAngles[3];
                    
                    long stepsCorrectionA = (long)(errorA * stepsPerDegree[3]);

                    if (AXIS_INVERT[3]) {
                        stepsCorrectionA = -stepsCorrectionA;
                    }

                    motors[3]->moveTo(motors[3]->currentPosition() + stepsCorrectionA);
                }
                
                lastAxisCorrectionTime[i] = currentMillis;
            }
        }
        
        // ===== 3. Wykonanie kroków (Motion Profiling) =====
        for(int i=0; i<5; i++) {
            motors[i]->run();
        }
        
        vTaskDelay(1); 
    }
}

long calculateMotorSteps(int axisIndex, float targetAxisAngle) {
    // 1. Oblicz różnicę kątową względem pozycji startowej
    float angleDifference = targetAxisAngle - START_ANGLES[axisIndex];
    
    // 2. Uwzględnij odwrócenie kierunku
    if (AXIS_INVERT[axisIndex]) {
        angleDifference = -angleDifference;
    }
    
    // 3. Oblicz liczbę obrotów OSI
    float axisRevolutions = angleDifference / 360.0;
    
    // 4. Oblicz liczbę obrotów SILNIKA (uwzględniając przekładnię)
    // Jeśli oś robi 1 obrót, silnik musi zrobić GEAR_RATIO obrotów
    float motorRevolutions = axisRevolutions * ENCODER_LEVER[axisIndex];
    
    // 5. Przelicz na obrót silnika
    long totalSteps = (long)(motorRevolutions * STEPS_PER_MOTOR_REV);
    
    return totalSteps;
}

// =========================================================================
// ------------------------------ SETUP I LOOP------------------------------
// =========================================================================

void setup() {
    Serial.begin(BAUD);
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    
    // Konfiguracja pinów
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
    
    delay(500);
    
    Serial.println("=== ESP32 Dual-Core Robot Control ===");
    
    // Utworzenie mutexu
    xMutex = xSemaphoreCreateMutex();
    if (xMutex == NULL) {
        Serial.println("❌ BŁĄD: Nie można utworzyć mutexu!");
        while(1);
    }
    
    // Kalibracja enkoderów
    Serial.println("Kalibracja enkoderów...");
    const char* axisNames[] = {"E", "Z", "Y", "A", "X"};
    
    for(int i = 0; i < 5; i++) {
        uint16_t rawReading = getEncoderRawAngle(ENCODER_CHANNEL[i]);
        
        if(rawReading == 0xFFFF) {
            Serial.printf("❌ BŁĄD: Enkoder %s nie odpowiada!\n", axisNames[i]);
            ENCODER_ZPOS[i] = 0;
        } else {
            ENCODER_ZPOS[i] = rawReading;
            Serial.printf("✓ Enkoder %s: raw=%d (start=%.1f°)\n", 
                          axisNames[i], ENCODER_ZPOS[i], START_ANGLES[i]);
        }
        
        lastRawAngle[i] = ENCODER_ZPOS[i];
        rotationCount[i] = 0;
        currentAngles[i] = START_ANGLES[i];
        targetAngles[i] = START_ANGLES[i];
    }
    
    // Uruchomienie tasków na osobnych rdzeniach
    Serial.println("Uruchamianie tasków...");
    
    // Task komunikacyjny na rdzeniu 0 (ten sam co loop())
    xTaskCreatePinnedToCore(
        communicationTask,   // Funkcja
        "Communication",     // Nazwa
        10000,              // Stack size (bajty)
        NULL,               // Parametr
        1,                  // Priorytet
        NULL,               // Handle
        0                   // Rdzeń 0
    );
    
    // Task sterowania na rdzeniu 1
    xTaskCreatePinnedToCore(
        motorControlTask,
        "MotorControl",
        10000,
        NULL,
        2,                  // Wyższy priorytet
        NULL,
        1                   // Rdzeń 1
    );
    
    Serial.println("✓ System uruchomiony!");
}

void loop() {
    // Loop może pozostać pusty - wszystko dzieje się w taskach
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

// =========================================================================
// ------------------------ FUNKCJE ENKODERÓW ------------------------------
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
        Serial.print(F("Error: Failed to select I2C Mux ch: "));
        Serial.println(channel);
        return 0xFFFF;
    }
    
    if (!isAS5600Available()) {
        Serial.print(F("Error: AS5600 sensor not found ch: "));
        Serial.println(channel);
        return 0xFFFF;
    }
    
    uint16_t angle;
    if (!readAS5600Raw(angle)) {
        Serial.print(F("Error: Failed to read data on ch: "));
        Serial.println(channel);
        return 0xFFFF;
    }
    
    return angle;
}

void updateRotationCount(int axisIndex, uint16_t currentRaw) {
    uint16_t lastRaw = lastRawAngle[axisIndex];
    
    if (lastRaw > 3000 && currentRaw < 1000) {
        rotationCount[axisIndex]++;
    }
    else if (lastRaw < 1000 && currentRaw > 3000) {
        rotationCount[axisIndex]--;
    }
    
    lastRawAngle[axisIndex] = currentRaw;
}