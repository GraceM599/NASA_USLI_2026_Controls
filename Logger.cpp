/*
 * ============================================================================
 * LOGGER - Implementation File
 * ============================================================================
 * Handles all data logging to SD card including debug messages and flight data
 */

#include "Logger.h"
#include "Debug.h"

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

LogEntry logBuffer[LOG_BUFFER_SIZE];
volatile int logWriteIndex = 0;
int logFlushIndex = 0;
File logFile;
File debugFile;
unsigned long lastFlush = 0;

// Timing reference (set from main code)
unsigned long* loggingStartTimePtr = nullptr;

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================

#define FLUSH_INTERVAL 2000000   // 2 Hz (in microseconds)

// ============================================================================
// INITIALIZATION
// ============================================================================

void setLoggingStartTime(unsigned long* startTimePtr) {
    loggingStartTimePtr = startTimePtr;
}

bool initializeLogger() {
    #ifdef DEBUG_NO_SD
    return true;  // Skip SD initialization if debugging without SD
    #endif
    
    DEBUG_PRINTLN("Initializing SD card...");

    if (!SD.begin(BUILTIN_SDCARD)) {
        DEBUG_PRINTLN("ERROR: SD card initialization failed!");
        return false;
    }

    // Remove old debug file if exists
    if (SD.exists(DEBUG_FILENAME)) {
        SD.remove(DEBUG_FILENAME);
    }

    // Initialize Debug file
    debugFile = SD.open(DEBUG_FILENAME, FILE_WRITE);
    if(!debugFile) {
        DEBUG_PRINTLN("ERROR: Could not open debug file!");
        return false;
    }

    // Write TXT header
    debugFile.println("=====================================================");
    debugFile.println("                 USLI Controls 2026                  ");
    debugFile.println("=====================================================");

    // Remove old log file if exists
    if (SD.exists(LOG_FILENAME)) {
        SD.remove(LOG_FILENAME);
    }

    // Initialize Log file
    logFile = SD.open(LOG_FILENAME, FILE_WRITE);
    if (!logFile) {
        DEBUG_PRINTLN("ERROR: Could not open log file!");
        return false;
    }

    // Write CSV header
    logFile.print("timestamp,");
    logFile.print("accelX,accelY,accelZ,");
    logFile.print("gyroX,gyroY,gyroZ,");
    logFile.print("baroAlt,baroPressure,baroTemp,");
    logFile.print("AGL,KFalt,KFvel,");
    logFile.print("gpsLat,gpsLon,gpsAlt,gpsSpeed,gpsSats,gpsFixOK,");
    logFile.println("phase");
    logFile.flush();
    
    DEBUG_PRINTLN("SD card initialized!");
    logDebugMessage("SD card initialized!");
    
    return true;
}

// ============================================================================
// DEBUG MESSAGE LOGGING
// ============================================================================

void logDebugMessage(const char* message, float value, int decimals) {
    #ifndef DEBUG_NO_SD
    if (debugFile && loggingStartTimePtr) {  // Check pointer is set
        unsigned long t = micros() - *loggingStartTimePtr;  // Use * to dereference
        float time_s = (float)t / 1000000.0f;
        debugFile.print("[");
        debugFile.print(time_s, 6);
        debugFile.print(" s] ");
        debugFile.print(message);
        debugFile.println(value, decimals);
        debugFile.flush();
    }
    #endif
}

void logDebugMessage(const char* message) {
    #ifndef DEBUG_NO_SD
    if (debugFile && loggingStartTimePtr) {  // Check pointer is set
        unsigned long t = micros() - *loggingStartTimePtr;  // Use * to dereference
        float time_s = (float)t / 1000000.0f;
        debugFile.print("[");
        debugFile.print(time_s, 6);
        debugFile.print(" s] ");
        debugFile.println(message);
        debugFile.flush();
    }
    #endif
}

// ============================================================================
// FLIGHT DATA LOGGING
// ============================================================================

void addLogEntry(State& currentState, uint8_t phase, float groundAltitude, unsigned long loggingStartTime) {
    LogEntry& entry = logBuffer[logWriteIndex];
    
    entry.timestamp = micros() - loggingStartTime;
    
    // IMU data (latest values)
    entry.accelX = currentState.accelX;
    entry.accelY = currentState.accelY;
    entry.accelZ = currentState.accelZ;
    entry.gyroX = currentState.gyroX;
    entry.gyroY = currentState.gyroY;
    entry.gyroZ = currentState.gyroZ;
    
    // Barometer data (latest values)
    entry.baroAltitude = currentState.baroAltitude;
    entry.baroPressure = currentState.baroPressure;
    entry.baroTemperature = currentState.baroTemperature;

    entry.altitudeAGL = currentState.baroAltitude - groundAltitude;

    // Kalman data
    entry.estimatedAltitude = currentState.estimatedAltitude;
    entry.estimatedVelocity = currentState.estimatedVelocity;
    
    // After Kalman data:
    entry.gpsLat = currentState.gpsLat;
    entry.gpsLon = currentState.gpsLon;
    entry.gpsAltitude = currentState.gpsAltitude;
    entry.gpsSpeed = currentState.gpsSpeed;
    entry.gpsSatellites = currentState.gpsSatellites;
    entry.gpsHasFix = currentState.gpsHasFix ? 1 : 0;

    // Flight phase
    entry.phase = phase;
    
    // Advance write index (circular buffer)
    logWriteIndex = (logWriteIndex + 1) % LOG_BUFFER_SIZE;
    
    // Safety check: buffer overflow
    if (logWriteIndex == logFlushIndex) {
        DEBUG_PRINTLN("WARNING: Log buffer overflow! Forcing flush...");
        flushLogBuffer(nullptr);  // Pass nullptr since we don't have phaseNames here
    }
}

void flushLogBuffer(const char* phaseNames[]) {
    if (!logFile) return;
    
    // Write all pending entries
    while (logFlushIndex != logWriteIndex) {
        LogEntry& entry = logBuffer[logFlushIndex];
        
        // Write as CSV
        logFile.print(entry.timestamp);
        logFile.print(',');
        logFile.print(entry.accelX, 6);
        logFile.print(',');
        logFile.print(entry.accelY, 6);
        logFile.print(',');
        logFile.print(entry.accelZ, 6);
        logFile.print(',');
        logFile.print(entry.gyroX, 6);
        logFile.print(',');
        logFile.print(entry.gyroY, 6);
        logFile.print(',');
        logFile.print(entry.gyroZ, 6);
        logFile.print(',');
        logFile.print(entry.baroAltitude, 3);
        logFile.print(',');
        logFile.print(entry.baroPressure, 2);
        logFile.print(',');
        logFile.print(entry.baroTemperature, 2);
        logFile.print(',');
        logFile.print(entry.altitudeAGL, 3);
        logFile.print(',');
        logFile.print(entry.estimatedAltitude, 3);
        logFile.print(',');
        logFile.print(entry.estimatedVelocity, 3);
        logFile.print(',');
        logFile.print(entry.gpsLat, 8);
        logFile.print(',');
        logFile.print(entry.gpsLon, 8);
        logFile.print(',');
        logFile.print(entry.gpsAltitude, 2);
        logFile.print(',');
        logFile.print(entry.gpsSpeed, 2);
        logFile.print(',');
        logFile.print(entry.gpsSatellites);
        logFile.print(',');
        logFile.print(entry.gpsHasFix);
        logFile.print(',');
        
        if (phaseNames != nullptr) {
            logFile.print(phaseNames[entry.phase]);
        } else {
            logFile.print(entry.phase);
        }
        logFile.println();
        
        logFlushIndex = (logFlushIndex + 1) % LOG_BUFFER_SIZE;
    }
    
    // Commit to SD card (this is the slow part)
    logFile.flush();
    lastFlush = micros();
    
    VERBOSE_PRINTLN("Log flushed to SD");
}

void checkFlushNeeded(const char* phaseNames[]) {
    unsigned long now = micros();
    
    // Calculate buffer usage
    int pending = (logWriteIndex - logFlushIndex + LOG_BUFFER_SIZE) % LOG_BUFFER_SIZE;
    bool bufferNearFull = pending > (LOG_BUFFER_SIZE * 0.75);
    bool timeToFlush = (now - lastFlush) >= FLUSH_INTERVAL;
    
    if ((bufferNearFull) && pending > 0) {
        flushLogBuffer(phaseNames);
    }
}

// ============================================================================
// CLEANUP
// ============================================================================

void closeLogger() {
    #ifndef DEBUG_NO_SD
    if (logFile) {
        logFile.close();
    }
    if (debugFile) {
        debugFile.close();
    }
    #endif
}
