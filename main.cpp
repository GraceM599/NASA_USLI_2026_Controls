/*
 * ============================================================================
 * APOGEE CONTROL SYSTEM (ACS) NASA USLI CONTROLS 2026
 * ============================================================================
 */

#include <Wire.h>
#include <SD.h>
#include <Adafruit_BMP3XX.h>
#include "MTi.h"
#include "State.h"
#include "Logger.h"
#include "Debug.h"
#include "PhaseManager.h"
#include "Sensors.h"
#include "GPS.h"
#include "Config.h"
#include "KalmanFilter.h"
#include "ApogeeController.h"
#include <ArduinoEigenDense.h>
using namespace Eigen;


// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// Pin definitions
#define IMU_DRDY_PIN 20
#define BUZZER_PIN 33
// #define SERVO_PIN 9            // TODO: Airbrake servo pin

// I2C addresses
#define IMU_ADDRESS 0x6B

// ============================================================================
// TIMING CONFIGURATION (microseconds)
// ============================================================================

//#define IMU_INTERVAL      5000      // 200 Hz
//#define BARO_INTERVAL     10000     // 100 Hz
//#define CONTROL_INTERVAL  136000     // every 34 ms
//static unsigned long lastRead = 0;
//#define GPS_INTERVAL      100000   // 10 Hz             // move this to config.h

unsigned long loggingStartTime = 0;

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

// State
State currentState = {0};

// Timing flags
volatile bool imuReady = false;
volatile bool baroReady = false;
volatile bool controlReady = false;
volatile bool gpsReady = false;

// Timers
//IntervalTimer imuTimer;
//IntervalTimer baroTimer;
IntervalTimer controlTimer;
IntervalTimer gpsTimer;

// Kalman Filter
KalmanFilter KF;
unsigned long lastKalmanTime = 0;

// GPS
// Create GPS object (using Serial8)
GPS gps(Serial8);

// PID
ApogeeController ACS;

// ============================================================================
// SET UP
// ============================================================================

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    
    Serial.begin(115200);
    while (!Serial && millis() < 3000);  // Wait max 3 seconds for serial
    DEBUG_PRINTLN("\n=== ROCKET FLIGHT COMPUTER ===");
    DEBUG_PRINTLN("Initializing...");
    
    // ========================================================================
    // Initialize SD Card
    // ========================================================================
    #ifndef DEBUG_NO_SD
    // Initialize SD Card and Logger
    if (!initializeLogger()) {
        while (1);  // Halt on failure
    }

    // Write TXT header
    debugFile.println("=====================================================");
    debugFile.println("                 USLI Controls 2026                  ");
    debugFile.println("=====================================================");

    loggingStartTime = micros();

    // Set the logging time reference for debug messages
    setLoggingStartTime(&loggingStartTime);

    // Initialize Logfile.csv
    if (SD.exists(LOG_FILENAME)) {
        SD.remove(LOG_FILENAME);
    }

    logFile = SD.open(LOG_FILENAME, FILE_WRITE);
    if (!logFile) {
        DEBUG_PRINTLN("ERROR: Could not open log file!");
        while (1);
    }

    // Write CSV header
    logFile.print("timestamp,");
    logFile.print("accelX,accelY,accelZ,");
    logFile.print("gyroX,gyroY,gyroZ,");
    logFile.print("baroAlt,baroPressure,baroTemp,");
    logFile.print("AGL,KFalt,KFvel,");
    logFile.print("gpsLat,gpsLon,gpsAlt,gpsSpeed,gpsSats,gpsFix,");
    logFile.println("phase");
    logFile.flush();
    
    DEBUG_PRINTLN("SD card initialized!");
    logDebugMessage("SD card initialized!");
    #endif

    // ========================================================================
    // Initialize I2C
    // ========================================================================
    Wire.begin();
    delay(100);
    
    // ========================================================================
    // Initialize IMU (MTi)
    // ========================================================================
    if (!initializeIMU()) {
        while (1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(200);
        }
    }
    
    // ========================================================================
    // Initialize Barometer (BMP390)
    // ========================================================================
    if (!initializeBarometer()) {
        while (1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(200);
        }
    }

    // ========================================================================
    // Initialize GPS (Ultimate GPS V3)
    // ========================================================================

    DEBUG_PRINTLN("Initializing GPS...");
    logDebugMessage("Initializing GPS...");
    gps.begin();

    /*
    // Wait for GPS fix with feedback
    DEBUG_PRINTLN("Waiting for GPS fix...");
    logDebugMessage("Waiting for GPS");
    bool hasFix = false;
    unsigned long gpsWaitStart = millis();
    unsigned long lastBeep = 0;

    while (!hasFix && (millis() - gpsWaitStart < 60000)) {  // Wait max 60 seconds
        gps.update();
        gps.updateState(currentState);
    
        if (currentState.gpsHasFix && currentState.gpsSatellites >= 4) {
            hasFix = true;
            DEBUG_PRINTLN("GPS FIX ACQUIRED!");
            DEBUG_PRINT("Satellites: "); DEBUG_PRINTLN(currentState.gpsSatellites);
            logDebugMessage("Satellites: ", currentState.gpsSatellites);
        
            // Success beep pattern: 3 quick beeps
            for (int i = 0; i < 3; i++) {
                tone(BUZZER_PIN, 2000, 100);
                delay(150);
            }
        } else {
            // Beep every 2 seconds while waiting
            if (millis() - lastBeep > 2000) {
                tone(BUZZER_PIN, 1000, 100);  // Low beep = still searching
                lastBeep = millis();
                DEBUG_PRINT("Searching... Sats: "); DEBUG_PRINTLN(currentState.gpsSatellites);
                logDebugMessage("Searching... Sats: ", currentState.gpsSatellites);
            }
        }
    
        delay(100);
    }

    if (!hasFix) {
        DEBUG_PRINTLN("WARNING: GPS fix not acquired, continuing anyway...");
        logDebugMessage("WARNING: GPS fix not acquired, continuing anyway...");
        // Warning beep pattern: 5 short beeps
        for (int i = 0; i < 5; i++) {
            tone(BUZZER_PIN, 500, 50);
            delay(100);
        }
    }
    
    */
    // Perform IMU tare (bias removal)
    #ifdef ENABLE_IMU_TARE
    performIMUTare();
    #endif

    // Read and store ground altitude (barometer "tare")
    #ifdef ENABLE_BARO_TARE
    performBarometerTare();
    #endif

    KF.initialize(0.0f);
    currentState.estimatedAltitude = 0.0f;
    currentState.estimatedVelocity = 0.0f;
    lastKalmanTime = micros();
    
    //GPS 
    currentState.gpsLat = 0.0f;
    currentState.gpsLon = 0.0f;
    currentState.gpsAltitude = 0.0f;
    currentState.gpsSpeed = 0.0f;
    currentState.gpsSatellites = 0;
    currentState.gpsHasFix = false;
    
    // Controller
    ACS.initialize();

    // ========================================================================
    // System Ready - Auto-arm and start
    // ========================================================================
    DEBUG_PRINTLN("\n=== SYSTEM READY ===");
    logDebugMessage("\n==================== SYSTEM READY ====================");
    
    // Auto-arm the system
    // Initialize Phase Manager and auto-arm
    initializePhaseManager();
    currentPhase = ARMED;
    phaseStartTime = micros();
    loggingStartTime = micros();
    DEBUG_PRINTLN(">>> SYSTEM ARMED <<<");
    DEBUG_PRINTLN("Waiting for liftoff detection...");
    logDebugMessage(">>> SYSTEM ARMED <<< Waiting for liftoff detection...");
    
    // Beep confirmation
    //tone(BUZZER_PIN, 2000, 100);
    //delay(1000);
    
    // ========================================================================
    // Start Hardware Timers
    // ========================================================================
    DEBUG_PRINTLN("Starting timers...");
    logDebugMessage("Starting timers...");
    
    //imuTimer.begin(imuISR, IMU_INTERVAL);
    //baroTimer.begin(baroISR, BARO_INTERVAL);
    //controlTimer.begin(controlISR, CONTROL_INTERVAL);
    //gpsTimer.begin(gpsISR, GPS_INTERVAL);
    
    DEBUG_PRINTLN("Flight computer running!");
    DEBUG_PRINTLN("========================================\n");
    logDebugMessage("Flight computer running!");
    
    digitalWrite(LED_BUILTIN, LOW);
}

// ============================================================================
// MAIN LOOP - Multi-rate sensor fusion & control
// ============================================================================

void loop() {
    //DEBUG_PRINT("Time: ");
    //DEBUG_PRINTLN2((micros() - loggingStartTime)/1000000.0, 6);
    
    readIMU(currentState);
    readBarometer(currentState);
    gps.update();
    gps.updateState(currentState);
    
    updateFlightPhase(currentState, groundAltitude, KF);

        // Run controller
        static float lastPrediction = 0.0f;
        float flapAngle = ACS.update(currentState, currentPhase);
    
        // Command servo (TODO)
    
        // Log when prediction changes (means controller ran)
        if (ACS.isControlActive()) {
            float newPrediction = ACS.getPredictedApogee();
        
            if (newPrediction != lastPrediction) {
                // LOG TO DEBUG FILE
                String logMsg = "CONTROLLER | Pred Apogee: " + String(newPrediction, 2) + 
                          " m | Error: " + String(ACS.getApogeeError(), 2) +
                          " m | Flap: " + String(flapAngle, 1) + 
                          " deg | Steps: " + String(ACS.getSimulationSteps());
                logDebugMessage(logMsg.c_str());
                Serial.println(logMsg); 
                lastPrediction = newPrediction;
            }
        }

        addLogEntry(currentState, currentPhase, groundAltitude, loggingStartTime);
   
        #ifdef DEBUG_SENSOR_READINGS
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 500) {               
            // TIME
            DEBUG_PRINT("Time: ");
            DEBUG_PRINT2((micros() - loggingStartTime)/1000000.0, 6);

            // IMU
            //DEBUG_PRINT("Accel: X="); DEBUG_PRINT2(currentState.accelX, 3);
            //DEBUG_PRINT(" Y="); DEBUG_PRINT2(currentState.accelY, 3);
            //DEBUG_PRINT(" accel Z = "); DEBUG_PRINT2(currentState.accelZ, 3);

            //DEBUG_PRINT("Gyro:  X="); DEBUG_PRINT2(currentState.gyroX, 3);
            //DEBUG_PRINT(" Y="); DEBUG_PRINT2(currentState.gyroY, 3);
            DEBUG_PRINT(" Z="); DEBUG_PRINT2(currentState.gyroZ, 3);

            // Barometer
            //DEBUG_PRINT(" Alt = "); DEBUG_PRINTLN2(currentState.baroAltitude, 2);
            DEBUG_PRINT(" AGL = "); DEBUG_PRINTLN2(currentState.baroAltitude - groundAltitude, 2);
            //DEBUG_PRINT(" m | Pressure="); DEBUG_PRINT2(currentState.baroPressure, 2);
            //DEBUG_PRINT(" hPa | Temp="); DEBUG_PRINT2(currentState.baroTemperature, 2);
            //DEBUG_PRINTLN(" C");

            // GPS
            
            //DEBUG_PRINT(" Lat: "); DEBUG_PRINT2(currentState.gpsLat, 6);
            //DEBUG_PRINT(" Lon: "); DEBUG_PRINT2(currentState.gpsLon, 6);
            //DEBUG_PRINT(" Alt: "); DEBUG_PRINT2(currentState.gpsAltitude, 1);
            //DEBUG_PRINT(" Spd: "); DEBUG_PRINT2(currentState.gpsSpeed, 1);
            //DEBUG_PRINT(" Sats: "); DEBUG_PRINT(currentState.gpsSatellites);
            //DEBUG_PRINT(" Fix: "); DEBUG_PRINTLN(currentState.gpsHasFix);
            
            
            // Flight phase
            DEBUG_PRINT("Phase: "); DEBUG_PRINTLN(phaseNames[currentPhase]);

            lastPrint = millis();
            
        }
        #endif

    
    // ========================================================================
    // PATH 5: SD CARD FLUSH (~1 Hz)
    // Periodic write to SD card (doesn't block control loop)
    // ========================================================================
    
    
    #ifndef DEBUG_NO_SD
    checkFlushNeeded(phaseNames);
    #endif
    //}
    //}

    //THIS MIGHT BE THE CAUSE OF THE SLOW

    // ========================================================================
    // PATH 6: SHUTDOWN DETECTION
    // ========================================================================
        
    if (currentPhase == LANDED) {
        // Force final flush
        DEBUG_PRINTLN("Flight complete. Flushing final data...");
        logDebugMessage("Flight complete. Flushing final data...");
        #ifndef DEBUG_NO_SD
        flushLogBuffer(phaseNames);
        
        // Close log file
        logFile.close();
        #endif
        
        // Beep to indicate complete
        //tone(BUZZER_PIN, 1000, 100);
        
        DEBUG_PRINTLN("Flight computer shutdown. Remove power to reset.");
        logDebugMessage("Flight computer shutdown. Remove power to reset.");
        
        // Stay in infinite loop (or enter low-power mode)
        while (1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(1000);
        }
    }
}
