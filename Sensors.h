/*
 * ============================================================================
 * SENSORS - Header File
 * ============================================================================
 * Handles all sensor initialization, reading, and calibration
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include "MTi.h"
#include "State.h"

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// Pin definitions
#define IMU_DRDY_PIN 20
#define GPS_SERIAL Serial8     // GPS connected to Serial8

// I2C addresses
#define IMU_ADDRESS 0x6B

// GPS Configuration
#define GPS_BAUD_INITIAL 9600
#define GPS_BAUD_OPERATING 115200
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F\r\n"
#define PMTK_SET_BAUD_115200 "$PMTK251,115200*1F\r\n"

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================

// Enable/disable sensor taring (calibration)
// #define ENABLE_IMU_TARE
// #define ENABLE_BARO_TARE

// ============================================================================
// EXTERNAL VARIABLES (declared in Sensors.cpp)
// ============================================================================

extern MTi *imu;
extern Adafruit_BMP3XX baro;
extern float imuTare[6];
extern bool imuTared;
extern float groundAltitude;

extern String nmeaBuffer;
extern unsigned long lastGPSRead;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

// Initialization
bool initializeIMU();
bool initializeBarometer();
bool initializeGPS();

// Calibration/Taring
void performIMUTare();
void performBarometerTare();

// Sensor Reading
void readIMU(State& currentState);
void readBarometer(State& currentState);
void readGPS(State& currentState);

#endif // SENSORS_H
