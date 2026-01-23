#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>

// ==================== Podstawowe ustawienia ====================
#define BAUD 115200
#define SIMULATION_MODE false

// ======================= Piny silników ======================
#define SERVO_PIN 13
Servo gripperServo;

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

#define SDA_PIN 22
#define SCL_PIN 21

AccelStepper motorX(AccelStepper::DRIVER, STEP_X, DIR_X);
AccelStepper motorY(AccelStepper::DRIVER, STEP_Y, DIR_Y);
AccelStepper motorA(AccelStepper::DRIVER, STEP_A, DIR_A);
AccelStepper motorZ(AccelStepper::DRIVER, STEP_Z, DIR_Z);
AccelStepper motorE(AccelStepper::DRIVER, STEP_E, DIR_E);

// ===================== Konfiguracja enkoderów [E, Z, Y, A, X] ======================
const float START_ANGLES[5] = {90.0, 90.0, 135.0, 135.0, 0.0}; 
const bool ENCODER_INVERT[] = {true, false, false, true, false};
const uint8_t ENCODER_CHANNEL[] = {4, 5, 6, 7, 3};
const float ENCODER_LEVER[] = {2.0, 3.6, 4.5, 4.5, 4.0};
const uint16_t ENCODER_ZPOS[] = {3777 + 45, 3982, 2763 + 135, 2415 + 40, 1800};
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

// ===================== Bufor komunikacyjny =======================
#define RX_BUF_SIZE 64
char inputBuffer[RX_BUF_SIZE];
uint8_t inputIdx = 0;

// ===================== Parametry sterowania ======================
const bool AXIS_INVERT[] = {false, true, true, true, true};
const float ANGLE_TOLERANCE = 0.05;
const unsigned long ENCODER_READ_INTERVAL = 5;
const unsigned long PYTHON_SEND_INTERVAL = 50;

// Deklaracje funkcji (prototypy)
void readSerialCommands();
void parsePythonCommand(String cmd);
void sendPositionToPython();
void readEncoders();
uint16_t getEncoderRawAngle(uint8_t channel); 
void updateRotationCount(int axisIndex, uint16_t currentRaw);
bool isStartPosition();
void calibration(const char* axisNames[]);

// ----------------------------- RDZEŃ 0 -----------------------------------
// ---------------------- KOMUNIKACJA I ENKODERY ---------------------------

void communicationTask(void *parameter) {
    unsigned long lastEncoderRead = 0;
    unsigned long lastPythonSend = 0;
    
    Serial.println("Rdzeń 0: Task komunikacyjny uruchomiony");
    
    while (true) {
        unsigned long now = millis();

        // Odczyt komend z Pythona 
        readSerialCommands();
        
        // Odczyt enkoderów
        if (now - lastEncoderRead >= ENCODER_READ_INTERVAL) {
            lastEncoderRead = now;
            readEncoders();
        }
        
        // Wysyłanie pozycji do Pythona
        if (now - lastPythonSend >= PYTHON_SEND_INTERVAL) {
            lastPythonSend = now;
            sendPositionToPython();
        }
        
        vTaskDelay(1); // Oddanie czasu WDT
    }
}   
  
void readSerialCommands() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        // Wykrycie końca linii (koniec ramki)
        if (c == '\n' || c == '\r') {
            if (inputIdx > 0) {
                inputBuffer[inputIdx] = '\0'; // Terminacja ciągu
                parsePythonCommand(inputBuffer);
                inputIdx = 0; // Reset indeksu
            }
        } else {
            // Zabezpieczenie przed przepełnieniem bufora
            if (inputIdx < RX_BUF_SIZE - 1) {
                inputBuffer[inputIdx] = c;
                inputIdx++;
            }
        }
    }
}

byte calculateChecksum(const char* data) {
    byte checksum = 0;
    while (*data) {
        checksum ^= *data++;
    }
    return checksum;
}

void parsePythonCommand(char* cmd) {
    // 1. Weryfikacja sumy kontrolnej
    char* starPtr = strrchr(cmd, '*');
    if (starPtr == NULL) return; // Brak separatora sumy

    *starPtr = '\0'; // Rozdzielenie payloadu od sumy (wpisanie null terminator w miejsce *)
    char* checksumStr = starPtr + 1; // Wskaźnik na tekst sumy kontrolnej

    byte calculatedChecksum = calculateChecksum(cmd);
    byte receivedChecksum = (byte) strtol(checksumStr, NULL, 16);
    
    if (calculatedChecksum != receivedChecksum) {
        Serial.println("Error: Checksum mismatch!"); 
        return; 
    }
    
    // 2. Parsowanie danych (tokenizacja)
    bool validCommand = false;
    float newTargets[5];
    float newServo;
    
    // Pobranie aktualnych celów, aby nie nadpisać niezmienianych osi
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
        memcpy(newTargets, (void*)targetAngles, sizeof(newTargets));
        newServo = targetServoAngle;
        xSemaphoreGive(xMutex);
    }
    
    char* token = strtok(cmd, ","); // Pobierz pierwszy token
    
    while (token != NULL) {
        // Format tokena np: "X12.5"
        if (strlen(token) >= 2) {
            char axis = token[0];
            float val = atof(token + 1); // Konwersja od drugiego znaku
            
            switch (axis) {
                case 'X': case 'x':
                    newTargets[4] = val; validCommand = true; break;
                case 'Y': case 'y': // Sprzężenie kinematyczne Y/A
                    newTargets[3] = val; 
                    newTargets[2] = val; 
                    validCommand = true; break;
                case 'Z': case 'z':
                    newTargets[1] = val; validCommand = true; break;
                case 'E': case 'e':
                    newTargets[0] = val; validCommand = true; break;
                case 'S': case 's':
                    newServo = val; validCommand = true; break;
            }
        }
        token = strtok(NULL, ","); // Pobierz kolejny token
    }
    
    // 3. Aktualizacja zmiennych sterujących
    if (validCommand) {
        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
            memcpy((void*)targetAngles, newTargets, sizeof(newTargets));
            targetServoAngle = newServo;
            newTargetAvailable = true;
            xSemaphoreGive(xMutex);
        }
        
        Serial.printf("Nowe cele: E=%.2f Z=%.2f Y=%.2f X=%.2f\n", 
                      newTargets[0], newTargets[1], newTargets[3], newTargets[4]);
    }
}

void sendPositionToPython() {
    float angles[5];
    float srv;
    
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
        memcpy(angles, (void*)currentAngles, sizeof(angles));
        srv = currentServoAngle;
        xSemaphoreGive(xMutex);
    }
    
    char txBuffer[128]; // Statyczny bufor nadawczy
    
    // Formatowanie ramki: <Sval,Eval,Zval,Yval,Aval,Xval>
    int len = snprintf(txBuffer, sizeof(txBuffer), 
             "<S%.2f,E%.2f,Z%.2f,Y%.2f,A%.2f,X%.2f>",
             srv, angles[0], angles[1], angles[2], angles[3], angles[4]);
             
    if (len > 0) {
        Serial.println(txBuffer);
    }
}

void readEncoders() {
    #if SIMULATION_MODE
        // SYMULACJA - wykorzystywana do programowania części python bez 
        // dostępu do manipulatora (konieczne jest podpięcie do esp) 
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

        // Pobranie danych
        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
            memcpy(tempCurrent, (void*)currentAngles, sizeof(tempCurrent));
            memcpy(tempTarget, (void*)targetAngles, sizeof(tempTarget));
            
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

        // Symulacja Serwa
        float servoSpeed = 240.0;
        float sDiff = tempServoTar - tempServoCur;
        
        if (abs(sDiff) < 0.1) {
            tempServoCur = tempServoTar;
        } else {
            float step = servoSpeed * dt;
            if (sDiff > 0) tempServoCur += min(sDiff, step);
            else tempServoCur += max(sDiff, -step);
        }

        // Zapisanie wyników symulacji
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

// ------------------------------ RDZEŃ 1 ----------------------------------
// ------------------------ STEROWANIE SILNIKAMI ---------------------------

void motorControlTask(void *parameter) {
    long targetSteps[5] = {0, 0, 0, 0, 0};
    float localTargetAngles[5];
    float localCurrentAngles[5];
    float stepsPerDegree[5];
    float localTargetServo = 0.0;
    float localCurrentServo = 0.0;

    // Zmienne stanu dla kompensacji osi X
    const float BACKLASH_X_DEG = 1.3; 
    static float lastTargetX = 0.0;
    static float activeBacklashX = 0.0;
    static bool isFirstRun = true;

    const float KP_MAIN = 1.0;          // Wzmocnienie proporcjonalne dla osi          
    const float KP_SLAVE_TRACKING = 1.0;// Wzmocnienie śledzenia osi Slave
    const float KP_SLAVE_SYNC = 0.15;   // Wzmocnienie błędu synchronizacji    
    
    const float SYNC_DEADBAND = 0.05;       // Strefa nieczułości dla błędu synchronicznego
    const float SYNC_MAX_CORR = 2.0;  // Siła korekty  
    
    unsigned long lastAxisCorrTime[5] = {0, 0, 0, 0, 0};
    const unsigned long CORR_INTERVAL = 20; // Okres próbkowania regulatora
    
    AccelStepper* motors[5] = {&motorE, &motorZ, &motorY, &motorA, &motorX};

    // Konfiguracja kinematyki i profilowanie
    const float STEPS_PER_REV = 200.0;  
    const float MICROSTEPS = 64.0;     
    const float STEPS_PER_MOTOR_REV = STEPS_PER_REV * MICROSTEPS; 
    const long MAX_CORR_STEPS = long(STEPS_PER_MOTOR_REV / 9); 
    float baseSpeed = 2.0 * STEPS_PER_MOTOR_REV;
    float baseAcceleration = baseSpeed / 8.0;
    
    for(int i=0; i<5; i++) {
        motors[i]->setMaxSpeed(baseSpeed * ENCODER_LEVER[i]);
        motors[i]->setAcceleration(baseAcceleration * ENCODER_LEVER[i]);
        stepsPerDegree[i] = (STEPS_PER_MOTOR_REV * ENCODER_LEVER[i]) / 360.0;
    }
    
    // Inicjalizacja zmiennej startowej dla X (żeby nie szarpnęło przy starcie)
    lastTargetX = START_ANGLES[4];

    Serial.println("Rdzeń 1: Task sterowania uruchomiony (Logika: Target Hysteresis)");
    
    while (true) {
        unsigned long currentMillis = millis();

        // ===== Pobieranie danych i obsługa backlash osi X =====
        if (xSemaphoreTake(xMutex, 10 / portTICK_PERIOD_MS)) {
            if (newTargetAvailable) {
                // Kopiujemy nowe cele
                memcpy(localTargetAngles, (void*)targetAngles, sizeof(localTargetAngles));
                newTargetAvailable = false;

                currentServoAngle = localCurrentServo;
                
                // Obsługa backlash osi X
                float newX = localTargetAngles[4];
                
                if (isFirstRun) {
                    lastTargetX = newX;
                    isFirstRun = false;
                }

                // Porównujemy nowy cel z poprzednim zapamiętanym
                if (abs(newX - lastTargetX) > 0.1) { // Tolerancja na float
                    if (newX > lastTargetX) {
                        activeBacklashX = BACKLASH_X_DEG;   // Ruch w stronę dodatnią
                    } 
                    else if (newX < lastTargetX) {
                        activeBacklashX = -BACKLASH_X_DEG;  // Ruch w stronę ujemną
                    }
                    lastTargetX = newX;
                }
                // ==========================================================
            }
            
            // Aktualizacja pozycji bieżącej
            memcpy(localCurrentAngles, (void*)currentAngles, sizeof(localCurrentAngles));
            xSemaphoreGive(xMutex);
        }
        
        // ===== Sterowanie silnikami krokowymi oraz serwomechanizmem =====
        if (abs(localTargetServo - localCurrentServo) > 0.5) {
            gripperServo.write((int)localTargetServo); // Zapis do sprzętu
            localCurrentServo = localTargetServo;      // Aktualizacja stanu
        }

        for (int i = 0; i < 5; i++) {
            if (i == 3) continue; // Pomiń Slave A
            
            if (currentMillis - lastAxisCorrTime[i] < CORR_INTERVAL) {
                continue;
            }

            // Obliczanie odchyłki od celu
            float effectiveTarget = localTargetAngles[i];
            if (i == 4) { //Oś X
                effectiveTarget += activeBacklashX;
            }
            float error = effectiveTarget - localCurrentAngles[i];

            // =============== Regulator P ===============
            if (abs(error) > ANGLE_TOLERANCE) {
                
                long stepsCorr = (long)(error * KP_MAIN * stepsPerDegree[i]);
                
                stepsCorr = constrain(stepsCorr, -MAX_CORR_STEPS, MAX_CORR_STEPS);

                if (AXIS_INVERT[i]) {
                    stepsCorr = -stepsCorr;
                }

                motors[i]->moveTo(motors[i]->currentPosition() + stepsCorr);
                
                // --- LOGIKA MASTER-SLAVE DLA OSI A ---
                if (i == 2) {
                    float errorA = localTargetAngles[2] - localCurrentAngles[3];
                    float syncDeviation = localCurrentAngles[2] - localCurrentAngles[3];
                    
                    float syncCorr = 0.0;

                    // Strefa nieczułości
                    if (abs(syncDeviation) > SYNC_DEADBAND) {
                        float effectiveError = (syncDeviation > SYNC_DEADBAND) ? 
                                               (syncDeviation - SYNC_DEADBAND) : 
                                               (syncDeviation + SYNC_DEADBAND);
                        syncCorr = effectiveError * KP_SLAVE_SYNC;
                        syncCorr = constrain(syncCorr, -SYNC_MAX_CORR, SYNC_MAX_CORR);
                    }
                    
                    // Całkowity sygnał sterujący dla Slave
                    float totalCorrA = errorA * KP_SLAVE_TRACKING + syncCorr;
                    long stepsCorrA = (long)(totalCorrA * stepsPerDegree[3]);
                    stepsCorrA = constrain(stepsCorrA, -MAX_CORR_STEPS, MAX_CORR_STEPS);

                    if (AXIS_INVERT[3]) stepsCorrA = -stepsCorrA;

                    motors[3]->moveTo(motors[3]->currentPosition() + stepsCorrA);
                }
                
                lastAxisCorrTime[i] = currentMillis;
            }
        }
        
        // Wykonanie kroków - MUSI BYĆ JAK NAJCZĘŚCIEJ
        for(int i=0; i<5; i++) {
            motors[i]->run();
        }
        
        yield(); // oddanie WDT
    }
}

// ---------------------------- SETUP I LOOP -------------------------------

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
    if (!isStartPosition()) {
        Serial.println("BŁĄD KRYTYCZNY: Robot nie jest w pozycji startowej (krańcówki nienaciśnięte)!");
        while(!isStartPosition()) vTaskDelay(1000);
    }

    const char* axisNames[] = {"E", "Z", "Y", "A", "X"};    
    
    /*
    // Kalibracja enkoderów
    Serial.println("Kalibracja enkoderów...");
    calibration(axisNames);
    */
    
    for(int i = 0; i < 5; i++) {
        Serial.printf("Enkoder %s: Ustawiona stała RAW=%d (start=%.1f°)\n", 
                      axisNames[i], ENCODER_ZPOS[i], START_ANGLES[i]);
        
        lastRawAngle[i] = ENCODER_ZPOS[i];
        rotationCount[i] = 0;
        currentAngles[i] = START_ANGLES[i];
        targetAngles[i] = START_ANGLES[i];
        vTaskDelay(100);
    }
    
    // Inicjalizacja serwo
    gripperServo.attach(SERVO_PIN); 
    gripperServo.write(0);

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

bool isStartPosition() {
    bool allLimitsPressed = true;
    /*if (digitalRead(LIMIT_Y_PIN) != LOW) {  
        Serial.println("Limit Y nieaktywny!");
        allLimitsPressed = false;
    }*/
    if (digitalRead(LIMIT_Z_PIN) != LOW) {
        Serial.println("Limit Z nieaktywny!");
        allLimitsPressed = false;
    }
    if (digitalRead(LIMIT_E_PIN) != LOW) {
        Serial.println("Limit E nieaktywny!");
        allLimitsPressed = false;
    }
    
    if (allLimitsPressed) {
        Serial.println("Pomyślna weryfikacja pozycji bazowej");
    }
    
    return allLimitsPressed;
}

void calibrateX() {
    Serial.println(F("\nRozpoczynanie sekwencji bazowania osi X"));
    
    motorX.setMaxSpeed(400);
    motorX.setAcceleration(500);

    // 1. Lewy ogranicznik
    Serial.println(F("Szukanie lewej krancowki: "));
    motorX.setSpeed(-200); 
    
    while (digitalRead(LIMIT_X_PIN) == HIGH) { 
        motorX.runSpeed();
        vTaskDelay(1); 
    }
    motorX.stop();
    vTaskDelay(500); 
    
    uint16_t rawMin = getEncoderRawAngle(ENCODER_CHANNEL[4]); 
    Serial.printf("MIN RAW: %d\n", rawMin);

    // Bezpieczny odjazd
    motorX.move(300);
    while (motorX.distanceToGo() != 0) {
        motorX.run();
        vTaskDelay(1);
    }

    // 2. Prawy ogranicznik
    Serial.println(F("Szukanie prawej krancowki: "));
    motorX.setSpeed(200); 
    
    while (digitalRead(LIMIT_X_PIN) == HIGH) {
        motorX.runSpeed();
        vTaskDelay(1);
    }
    motorX.stop();
    vTaskDelay(500);
    
    uint16_t rawMax = getEncoderRawAngle(ENCODER_CHANNEL[4]);
    Serial.printf("MAX RAW: %d\n", rawMax);

    // 3. Obliczenie środka z uwzględnieniem przejścia przez zero
    long virtualMax = rawMax;

    if (virtualMax < rawMin) {
        virtualMax += 4096; // Korekta obrotu
    }

    long centerRaw = (long(rawMin) + virtualMax) / 2;

    // Normalizacja do zakresu 12-bit
    if (centerRaw >= 4096) {
        centerRaw -= 4096;
    }
    
    Serial.printf("[WYNIK] Obliczony XOffset: %ld\n", centerRaw);
}

void calibration(const char* axisNames[]) {
    unsigned long lastPrint = 0;
    
    Serial.println(F("Wyslij 'x' aby uruchomic bazowanie osi X."));
    
    // Statyczny bufor dla logów - wystarczający na 5 osi
    char logBuffer[128]; 

    while (true) { 
        if (Serial.available() > 0) {
            char cmd = Serial.read();
            if (cmd == 'x' || cmd == 'X') {
                calibrateX();
            }
        }

        if (millis() - lastPrint >= 500) {
            lastPrint = millis();
            
            // Wyczyszczenie bufora i przygotowanie wskaźnika przesunięcia
            int offset = 0;
            logBuffer[0] = '\0';

            for (int i = 0; i < 5; i++) {
                uint16_t rawAngle = getEncoderRawAngle(ENCODER_CHANNEL[i]); 
                
                // Bezpieczne dopisywanie do bufora (append)
                if (rawAngle != 0xFFFF) {
                    offset += snprintf(logBuffer + offset, sizeof(logBuffer) - offset, 
                                       "%s:%d ", axisNames[i], rawAngle);
                } else {
                    offset += snprintf(logBuffer + offset, sizeof(logBuffer) - offset, 
                                       "%s:ERROR ", axisNames[i]);
                }
                
                // Zabezpieczenie przed wyjściem poza bufor
                if (offset >= sizeof(logBuffer)) break; 
            }
            Serial.println(logBuffer);
        }
        vTaskDelay(10);
    }
}

void loop() { // Wszystko dzieje się w taskach
 // Zostawiamy puste
} 

// ------------------------ FUNKCJE ENKODERÓW ------------------------------

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