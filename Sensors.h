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

// I2C addresses
#define IMU_ADDRESS 0x6B

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

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

// Initialization
bool initializeIMU();
bool initializeBarometer();

// Calibration/Taring
void performIMUTare();
void performBarometerTare();

// Sensor Reading
void readIMU(State& currentState);
void readBarometer(State& currentState);

#endif // SENSORS_H
