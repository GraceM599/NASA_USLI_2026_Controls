/*
 * ============================================================================
 * LOGGER - Header File
 * ============================================================================
 * Handles all data logging to SD card including debug messages and flight data
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include <SD.h>
#include "State.h"

// ============================================================================
// LOGGING CONFIGURATION
// ============================================================================

#define LOG_BUFFER_SIZE 200         // 2 seconds at 100 Hz
#define LOG_FILENAME "flight_log.csv"
#define DEBUG_FILENAME "debug_log.txt"

// ============================================================================
// DATA STRUCTURES
// ============================================================================

// Complete log entry (for SD card)
struct LogEntry {
    uint32_t timestamp;
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float baroAltitude, baroPressure, baroTemperature;
    float altitudeAGL;
    float estimatedAltitude, estimatedVelocity;
    float gpsLat, gpsLon, gpsAltitude, gpsSpeed;
    uint8_t gpsSatellites;
    uint8_t gpsHasFix;
    uint8_t phase;
} __attribute__((packed));

// ============================================================================
// EXTERNAL VARIABLES (declared in Logger.cpp)
// ============================================================================

extern LogEntry logBuffer[LOG_BUFFER_SIZE];
extern volatile int logWriteIndex;
extern int logFlushIndex;
extern File logFile;
extern File debugFile;
extern unsigned long lastFlush;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

// Initialize SD card and log files
bool initializeLogger();

// Set logging start time reference (call this after setting loggingStartTime in main)
void setLoggingStartTime(unsigned long* startTimePtr);

// Debug message logging
void logDebugMessage(const char* message, float value, int decimals = 4);
void logDebugMessage(const char* message);

// Flight data logging (manual parameter passing)
void addLogEntry(State& currentState, uint8_t phase, float groundAltitude, unsigned long loggingStartTime);
void flushLogBuffer(const char* phaseNames[]);
void checkFlushNeeded(const char* phaseNames[]);

// Cleanup
void closeLogger();

#endif // LOGGER_H
