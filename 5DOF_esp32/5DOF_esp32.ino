#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>

// ==================== Podstawowe ustawienia ====================
#define BAUD 115200
#define SIMULATION_MODE true

// ======================= Piny silników ======================
#define SERVO_PIN 13

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

#define SDA_PIN 21
#define SCL_PIN 22

AccelStepper motorX(AccelStepper::DRIVER, STEP_X, DIR_X);
AccelStepper motorY(AccelStepper::DRIVER, STEP_Y, DIR_Y);
AccelStepper motorA(AccelStepper::DRIVER, STEP_A, DIR_A);
AccelStepper motorZ(AccelStepper::DRIVER, STEP_Z, DIR_Z);
AccelStepper motorE(AccelStepper::DRIVER, STEP_E, DIR_E);

// ===================== Konfiguracja enkoderów [E, Z, Y, A, X] ======================
const float START_ANGLES[5] = {90.0, 90.0, 135.0, 135.0, 0.0}; 
const bool ENCODER_INVERT[] = {true, false, true, true, false};
const uint8_t ENCODER_CHANNEL[] = {4, 5, 6, 7, 3};
const float ENCODER_LEVER[] = {2.0, 3.6, 4.5, 4.5, 1.0};
uint16_t ENCODER_ZPOS[] = {0, 0, 0, 0, 0};
int16_t rotationCount[] = {0, 0, 0, 0, 0};
uint16_t lastRawAngle[] = {0, 0, 0, 0, 0};
const float angleConst = 360.0 / 4096.0;

// ===================== Zmienne współdzielone (z mutex) ======================
SemaphoreHandle_t xMutex;
volatile float currentAngles[5] = {90.0, 90.0, 135.0, 135.0, 0.0};
volatile float targetAngles[5] = {90.0, 90.0, 135.0, 135.0, 0.0};
volatile float targetServoAngle = 0.0; // Stan serwonapędu
volatile float currentServoAngle = 0.0;
volatile bool newTargetAvailable = true;

// ===================== Parametry sterowania ======================
const bool AXIS_INVERT[] = {false, false, false, false, false};
const float ANGLE_TOLERANCE = 0.05;
const unsigned long ENCODER_READ_INTERVAL = 20;
const unsigned long PYTHON_SEND_INTERVAL = 50;

// Bufory komunikacji (tylko rdzeń 0)
String serialBuffer = "";

// =========================================================================
// ----------------------------- RDZEŃ 0 -----------------------------------
// ---------------------- KOMUNIKACJA I ENKODERY ---------------------------
// =========================================================================

void communicationTask(void *parameter) {
    unsigned long lastEncoderRead = 0;
    unsigned long lastPythonSend = 0;
    
    Serial.println("Rdzeń 0: Task komunikacyjny uruchomiony");
    
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

byte calculateChecksum(String data) {
    // Funkcja pomocnicza do obliczania sumy kontrolnej XOR
    byte checksum = 0;
    for (int i = 0; i < data.length(); i++) {
        checksum ^= data.charAt(i);
    }
    return checksum;
}

void parsePythonCommand(String cmd) {
    cmd.trim();
    
    // Weryfikacja integralności danych (Suma kontrolna)
    int starIdx = cmd.lastIndexOf('*');
    if (starIdx == -1) return; // Odrzuć ramkę, jeśli brak separatora sumy kontrolnej

    String payload = cmd.substring(0, starIdx);
    String receivedChecksumHex = cmd.substring(starIdx + 1);
    
    byte calculatedChecksum = calculateChecksum(payload);
    byte receivedChecksum = (byte) strtol(receivedChecksumHex.c_str(), NULL, 16);
    
    if (calculatedChecksum != receivedChecksum) {
        Serial.println("Error: Checksum mismatch!"); 
        return; // Odrzuć ramkę w przypadku błędu transmisji
    }
    
    int startIdx = 0;
    bool validCommand = false;
    float newTargets[5];
    float newServo;
    
    // Kopiuj aktualne cele
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
        // Bezpieczny dostęp do danych współdzielonych tworząc lokalne kopie
        memcpy(newTargets, (void*)targetAngles, sizeof(newTargets));
        newServo = targetServoAngle;
        xSemaphoreGive(xMutex); // Zwolnienie blokady
    }
    
    // Parsowanie (Operujemy na zweryfikowanym 'payload', a nie surowym 'cmd')
    while (startIdx < payload.length()) {
        int commaIdx = payload.indexOf(',', startIdx);
        String axisData;
        
        if (commaIdx == -1) {
            axisData = payload.substring(startIdx);
            startIdx = payload.length();
        } else {
            axisData = payload.substring(startIdx, commaIdx);
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
            case 'S': case 's':
                newServo = relativeAngle;       
                validCommand = true; 
                break;
        }
    }
    
    // Zapisz nowe cele (chronione mutexem)
    if (validCommand) {
        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
            memcpy((void*)targetAngles, newTargets, sizeof(newTargets));
            targetServoAngle = newServo;
            newTargetAvailable = true;
            xSemaphoreGive(xMutex);
        }
        
        Serial.println("Nowe kąty docelowe:");
        Serial.printf("   E=%.2f° Z=%.2f° Y=%.2f° X=%.2f°\n", 
                      newTargets[0], newTargets[1], newTargets[3], newTargets[4]);
    }
}

void sendPositionToPython() {
    float angles[5];
    float srv;
    
    // Bezpieczne skopiowanie danych
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
        memcpy(angles, (void*)currentAngles, sizeof(angles));
        srv = currentServoAngle;
        xSemaphoreGive(xMutex);
    }
    
    String frame = "<";
    frame += "S" + String(srv, 2) + ",";
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
        float dt = (now - lastUpdate) / 1000.0;
        
        // Zabezpieczenie przed pierwszym uruchomieniem lub błędem zegara
        if (dt <= 0 || dt > 1.0) {
            lastUpdate = now;
            return;
        }
        lastUpdate = now;

        float tempCurrent[5], tempTarget[5];
        float tempServoCur, tempServoTar;

        // 1. Pobranie danych
        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
            memcpy(tempCurrent, (void*)currentAngles, sizeof(tempCurrent));
            memcpy(tempTarget, (void*)targetAngles, sizeof(tempTarget));
            
            // [POPRAWKA] Pobranie stanu serwa
            tempServoCur = currentServoAngle;
            tempServoTar = targetServoAngle;
            
            xSemaphoreGive(xMutex);
        }

        // 2. Symulacja silników krokowych
        float stepperSpeed = 60.0;
        for (int i = 0; i < 5; i++) {
            float diff = tempTarget[i] - tempCurrent[i];
            if (abs(diff) < 0.1) {
                tempCurrent[i] = tempTarget[i];
            } else {
                float step = stepperSpeed * dt;
                if (diff > 0) tempCurrent[i] += min(diff, step);
                else tempCurrent[i] += max(diff, -step);
            }
        }

        // 3. Symulacja Serwa
        float servoSpeed = 240.0;
        float sDiff = tempServoTar - tempServoCur;
        
        if (abs(sDiff) < 0.1) {
            tempServoCur = tempServoTar;
        } else {
            float step = servoSpeed * dt;
            if (sDiff > 0) tempServoCur += min(sDiff, step);
            else tempServoCur += max(sDiff, -step);
        }

        // 4. Zapisanie wyników symulacji
        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
            memcpy((void*)currentAngles, tempCurrent, sizeof(tempCurrent));
            currentServoAngle = tempServoCur;
            xSemaphoreGive(xMutex);
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
// ------------------------------ RDZEŃ 1 ----------------------------------
// ------------------------ STEROWANIE SILNIKAMI ---------------------------
// =========================================================================

void motorControlTask(void *parameter) {
    long targetSteps[5] = {0, 0, 0, 0, 0};
    float localTargetAngles[5];
    float localCurrentAngles[5];
    float stepsPerDegree[5];
    
    // Współczynniki regulatora P
    const float KP_MAIN = 1.0;              // Standardowe osie
    const float KP_SLAVE_TRACKING = 1.0;    // Slave śledzi cel Mastera
    const float KP_SLAVE_SYNC = 0.15;       // Łagodna synchronizacja
    
    // Limity bezpieczeństwa
    const long MAX_CORRECTION_STEPS = 500;
    
    // Tolerancja desynchronizacji Master-Slave
    const float SYNC_DEADBAND = 0.3;        // Ignoruj błędy sync < 0.3°stopnia
    const float SYNC_MAX_CORRECTION = 2.0;  // Maksymalna korekta sync za cykl [stopnie]
    
    // Strefa krytyczna (wokół pionu)
    const float CRITICAL_ZONE_MIN = 80.0;
    const float CRITICAL_ZONE_MAX = 110.0;
    const float CRITICAL_ZONE_DAMPING = 0.5; // Redukcja sync do 50% w strefie krytycznej
    
    // Niezależne timery dla każdej osi w celu uniknięcia oscylacji
    unsigned long lastAxisCorrectionTime[5] = {0, 0, 0, 0, 0};
    const unsigned long CORRECTION_INTERVAL = 50; 
    
    // Wskaźniki na obiekty silników
    AccelStepper* motors[5] = {&motorE, &motorZ, &motorY, &motorA, &motorX};

    // Konfiguracja silników
    const float STEPS_PER_REV = 200.0;  // Silnik 1.8 stopnia
    const float MICROSTEPS = 16.0;     // Ustawienie sterownika
    const float STEPS_PER_MOTOR_REV = STEPS_PER_REV * MICROSTEPS; // 3200 kroków/obrót silnika
    float baseSpeed = 50.0 * MICROSTEPS;
    float baseAcceleration = baseSpeed / 4.0;
    
    for(int i=0; i<5; i++) {
        motors[i]->setMaxSpeed(baseSpeed * ENCODER_LEVER[i]);
        motors[i]->setAcceleration(baseAcceleration * ENCODER_LEVER[i]);
        stepsPerDegree[i] = (STEPS_PER_MOTOR_REV * ENCODER_LEVER[i]) / 360.0;
    }
    
    Serial.println("Rdzeń 1: Task sterowania uruchomiony");
    
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
            
            // Sprawdzanie CORRECTION_INTERVAL
            if (currentMillis - lastAxisCorrectionTime[i] < CORRECTION_INTERVAL) {
                continue;
            }

            // Obliczenie uchybu: e(t) = wartość_zadana - wartość_mierzona
            float error = localTargetAngles[i] - localCurrentAngles[i];

            // Weryfikacja strefy nieczułości (deadband)
            if (abs(error) > ANGLE_TOLERANCE) {
                
                long stepsCorrection = (long)(error * KP_MAIN * stepsPerDegree[i]);
                
                // Limitowanie korekty
                stepsCorrection = constrain(stepsCorrection, -MAX_CORRECTION_STEPS, MAX_CORRECTION_STEPS);

                if (AXIS_INVERT[i]) {
                    stepsCorrection = -stepsCorrection;
                }

                // Aplikacja ruchu dla silnika bieżącego
                motors[i]->moveTo(motors[i]->currentPosition() + stepsCorrection);
                
                // --- LOGIKA MASTER-SLAVE DLA OSI A (Z DETEKCJĄ DESYNCHRONIZACJI) ---
                if (i == 2) {
                    // Obliczenie uchybu dla A z detekcją desynchronizacji
                    float errorA = localTargetAngles[2] - localCurrentAngles[3];
                    float syncDeviation = localCurrentAngles[2] - localCurrentAngles[3];
                    
                    // Dead-zone dla synchronizacji
                    float syncCorrection = 0.0;
                    if (abs(syncDeviation) > SYNC_DEADBAND) {
                        // Ogranicz korektę sync do maksymalnej wartości
                        syncCorrection = constrain(
                            syncDeviation * KP_SLAVE_SYNC, 
                            -SYNC_MAX_CORRECTION, 
                            SYNC_MAX_CORRECTION
                        );
                    }
                    
                    // Tłumienie w strefie krytycznej (wokół pionu)
                    float currentMasterAngle = localCurrentAngles[2];
                    if (currentMasterAngle >= CRITICAL_ZONE_MIN && 
                        currentMasterAngle <= CRITICAL_ZONE_MAX) {
                        syncCorrection *= CRITICAL_ZONE_DAMPING; // Redukcja do 50%
                    }
                    
                    // Slave śledzi cel + łagodna synchronizacja
                    float totalCorrectionA = errorA * KP_SLAVE_TRACKING + syncCorrection;
                    
                    // Konwersja na kroki
                    long stepsCorrectionA = (long)(totalCorrectionA * stepsPerDegree[3]);
                    
                    // Limitowanie Slave
                    stepsCorrectionA = constrain(stepsCorrectionA, -MAX_CORRECTION_STEPS, MAX_CORRECTION_STEPS);

                    // Aplikacja ruchu dla A
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

// =========================================================================
// ---------------------------- SETUP I LOOP -------------------------------
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
        Serial.println("BŁĄD: Nie można utworzyć mutexu!");
        while(1);
    }
    
    // Kalibracja enkoderów
    Serial.println("Kalibracja enkoderów...");

    

    // === funkcja weryfikująca pozycję startową ===
    if (!verifyHomingPosition()) {
        Serial.println("BŁĄD KRYTYCZNY: Robot nie jest w pozycji startowej (krańcówki nienaciśnięte)!");
        while(true) vTaskDelay(100); 
    }

    // Kalibracja enkoderów (po weryfikacji)
    Serial.println("Kalibracja enkoderów...");
    const char* axisNames[] = {"E", "Z", "Y", "A", "X"};
    calibration(axisNames);
    
    for(int i = 0; i < 5; i++) {
        Serial.printf("Enkoder %s: Ustawiona stała RAW=%d (start=%.1f°)\n", 
                      axisNames[i], ENCODER_ZPOS[i], START_ANGLES[i]);
        
        lastRawAngle[i] = ENCODER_ZPOS[i];
        rotationCount[i] = 0;
        currentAngles[i] = START_ANGLES[i];
        targetAngles[i] = START_ANGLES[i];
    }
    
    // Uruchomienie tasków na osobnych rdzeniach
    Serial.println("Uruchamianie tasków...");
    
    // wątek na rdzeniu 0
    xTaskCreatePinnedToCore(
        communicationTask,  // Funkcja
        "Communication",    // Nazwa
        10000,              // Stack size (bajty)
        NULL,               // Parametr
        1,                  // Priorytet
        NULL,               // Handle
        0                   // Rdzeń 0
    );
    
    // wątek na rdzeniu 1
    xTaskCreatePinnedToCore(
        motorControlTask,
        "MotorControl",
        10000,
        NULL,
        2, 
        NULL,
        1
    );
    
    Serial.println("System uruchomiony!");
}

bool verifyHomingPosition() {
    bool allLimitsPressed = true;
    if (digitalRead(LIMIT_X_PIN) != LOW) {
        Serial.println("Limit X nieaktywny!");
        allLimitsPressed = false;
    }
    if (digitalRead(LIMIT_Y_PIN) != LOW) {
        Serial.println("Limit Y nieaktywny!");
        allLimitsPressed = false;
    }
    if (digitalRead(LIMIT_Z_PIN) != LOW) {
        Serial.println("Limit Z nieaktywny!");
        allLimitsPressed = false;
    }
    if (digitalRead(LIMIT_E_PIN) != LOW) {
        Serial.println("Limit E nieaktywny!");
        allLimitsPressed = false;
    }
    
    if (allLimitsPressed) {
        Serial.println("Pomyślna weryfikacja pozycji bazowej. Można kalibrować.");
    }
    
    return allLimitsPressed;
}

void calibration(char* axisNames) {
    unsigned long lastPrint = 0;
    
    while (true) { 
        if (millis() - lastPrint >= 500) {
            lastPrint = millis();
            String output = "";
            
            for (int i = 0; i < 5; i++) {
                // Wywołanie funkcji odczytującej surowy kąt
                uint16_t rawAngle = getEncoderRawAngle(ENCODER_CHANNEL[i]); 
                
                if (rawAngle != 0xFFFF) {
                    output += String(axisNames[i]) + ":" + String(rawAngle) + " ";
                } else {
                    output += String(axisNames[i]) + ": ERROR ";
                }
            }
            
            Serial.println(output);
        }
    }
}

void loop() {} // Wszystko dzieje się w taskach

// =========================================================================
// ------------------------ FUNKCJE ENKODERÓW ------------------------------
// =========================================================================

bool selectI2CChannel(uint8_t channel) {
    Wire.beginTransmission(0x70);
    Wire.write(1 << channel);
    return (Wire.endTransmission() == 0);
}

bool isAS5600Available() {
    Wire.beginTransmission(0x36);
    return (Wire.endTransmission(true) == 0);
}

bool readAS5600Raw(uint16_t &angle) {
    Wire.beginTransmission(0x36);
    Wire.write(0x0C);
    if (Wire.endTransmission(false) != 0) return false;

    Wire.requestFrom(0x36, 2);
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