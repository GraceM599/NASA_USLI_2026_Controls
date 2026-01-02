#ifndef SENSORS_H
#define SENSORS_H

#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include "MTi.h"
#include "State.h"

#define IMU_DRDY_PIN 20
#define IMU_ADDRESS 0x6B

extern MTi *imu;
extern Adafruit_BMP3XX baro;
extern float groundAltitude;

bool initializeIMU();
void performIMUTare();
void readIMU(State& currentState);

bool initializeBarometer();
void performBarometerTare();
void readBarometer(State& currentState);

#endif
