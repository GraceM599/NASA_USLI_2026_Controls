/*
 * ============================================================================
 * SENSORS - Simple Direct Read (Like Python)
 * ============================================================================
 * Just read the sensors when you need them. No timers, no ISRs, no cache.
 */

#include "Sensors.h"
#include "Debug.h"
#include "Logger.h"
#include <cmath>
// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

MTi *imu = nullptr;
Adafruit_BMP3XX baro;

// ============================================================================
// IMU STATE
// ============================================================================

float imuTare[9] = {0.0f};
bool imuTared = false;
float groundAltitude = 0.0f;

// ============================================================================
// IMU INITIALIZATION
// ============================================================================

bool initializeIMU() {
    DEBUG_PRINTLN("Initializing IMU...");
    logDebugMessage("Initializing IMU...");

    pinMode(IMU_DRDY_PIN, INPUT);
    imu = new MTi(IMU_ADDRESS, IMU_DRDY_PIN);

    if (!imu->detect(1000)) {
        DEBUG_PRINTLN("ERROR: IMU not detected!");
        return false;
    }

    imu->goToConfig();
    delay(100);
    imu->requestDeviceInfo();
    delay(100);
    imu->configureOutputs();
    delay(100);
    imu->goToMeasurement();
    delay(100);
    delay(500);

    DEBUG_PRINTLN("IMU initialized!");
    logDebugMessage("IMU initialized!");
    return true;
}

// ============================================================================
// IMU TARE
// ============================================================================

void performIMUTare() {
    DEBUG_PRINTLN("Performing IMU tare...");
    logDebugMessage("Performing IMU tare...");

    float sum[6] = {0};
    //default euler angles
    float eulerSum[3] = {0};

    int samples = 0;
    unsigned long start = millis();

    while (millis() - start < 3000) {
        if (digitalRead(IMU_DRDY_PIN)) {
            imu->readMessages();
            float *acc  = imu->getAcceleration();
            float *gyro = imu->getRateOfTurn();

            sum[0] += acc[0];
            sum[1] += acc[1];
            sum[2] += acc[2];
            sum[3] += gyro[0];
            sum[4] += gyro[1];
            sum[5] += gyro[2];


            float *euler = imu->getEulerAngles();
            eulerSum[0] += euler[0];
            eulerSum[1] += euler[1];
            eulerSum[2] += euler[2];
            samples++;
        }
        delay(10);
    }

    if (samples > 0) {
        for (int i = 0; i < 6; i++) {
            imuTare[i] = sum[i] / samples;
        }
        for (int i = 0; i<3; i++) {
            imuTare[i+6] = eulerSum[i] / samples;
        }
        imuTared = true;
        DEBUG_PRINTLN("IMU tare complete!");
        DEBUG_PRINT("Bias: ax="); DEBUG_PRINT2(imuTare[0], 4);
        DEBUG_PRINT(" ay="); DEBUG_PRINT2(imuTare[1], 4);
        DEBUG_PRINT(" az="); DEBUG_PRINTLN2(imuTare[2], 4);
        DEBUG_PRINT("Samples collected: "); DEBUG_PRINTLN(samples);

        // log debug message
        logDebugMessage("IMU tare complete!");
        logDebugMessage("Bias: ax=", imuTare[0], 4);
        logDebugMessage("      ay=", imuTare[1], 4);
        logDebugMessage("      az=", imuTare[2], 4);
        logDebugMessage("Samples Collected:  ", samples);

    } else {
        DEBUG_PRINTLN("WARNING: No valid IMU samples collected!");
        logDebugMessage("WARNING: No valid IMU samples collected!");
    }
}

// ============================================================================
// READ IMU - Just read it when called
// ============================================================================

void readIMU(State &currentState) {
    if (!imu) return;
    
    // Only read if data is ready
    if (!digitalRead(IMU_DRDY_PIN)) return;
    
    imu->readMessages();
    
    float *acc  = imu->getAcceleration();
    float *gyro = imu->getRateOfTurn();
    float *eulerAngle = imu->getEulerAngles();
    if (imuTared) {
        currentState.accelX = acc[0] - imuTare[0];
        currentState.accelY = acc[1] - imuTare[1];
        currentState.accelZ = acc[2] - imuTare[2];
        currentState.gyroX  = gyro[0] - imuTare[3];
        currentState.gyroY  = gyro[1] - imuTare[4];
        currentState.gyroZ  = gyro[2] - imuTare[5];

        //imuTare[6-8] has the default values for this stuff but idk if its needed rn
        currentState.roll = eulerAngle[0];
        currentState.pitch = eulerAngle[1];
        currentState.yaw = eulerAngle[2];
    } else {
        currentState.accelX = acc[0];
        currentState.accelY = acc[1];
        currentState.accelZ = acc[2];
        currentState.gyroX  = gyro[0];
        currentState.gyroY  = gyro[1];
        currentState.gyroZ  = gyro[2];

        currentState.roll = eulerAngle[0];
        currentState.pitch = eulerAngle[1];
        currentState.yaw = eulerAngle[2];
    }
}
//not sure where this should go
bool tooPitched(State &currentState) {
    //some condition to see if the roll/pitch/yaw of the imu is a problem/past threshold

    const float MAX_SAFE_PITCH = 30.0f;
    if (std::abs(currentState.pitch) > MAX_SAFE_PITCH) {
        return true; //if true then stop the servo
    }
    //imu_tare[index 3-5] has the rates if thats needed.
    return false;
}
//To display the angles read
void displayEulerAngles(State &currentState) {
    DEBUG_PRINTLN("Roll: ", currentState.roll);
    logDebugMessage("Roll: ", currentState.roll);
    DEBUG_PRINTLN("Pitch: ", currentState.pitch);
    logDebugMessage("Pitch: ", currentState.pitch);
    DEBUG_PRINTLN("Yaw: ", currentState.yaw);
    logDebugMessage("Yaw: ", currentState.yaw);
}
// ============================================================================
// BAROMETER INITIALIZATION
// ============================================================================

bool initializeBarometer() {
    DEBUG_PRINTLN("Initializing Barometer...");
    logDebugMessage("Initializing Barometer...");

    if (!baro.begin_I2C()) {
        DEBUG_PRINTLN("ERROR: BMP390 not detected!");
        return false;
    }

    baro.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
    baro.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    baro.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    baro.setOutputDataRate(BMP3_ODR_100_HZ);

    DEBUG_PRINTLN("Barometer initialized!");
    logDebugMessage("Barometer initialized!");
    return true;
}

// ============================================================================
// BAROMETER TARE
// ============================================================================

void performBarometerTare() {
    DEBUG_PRINTLN("Reading ground altitude...");
    logDebugMessage("Reading ground altitude...");

    float altSum = 0.0f;
    int altSamples = 0;

    const int DISCARD_READINGS = 10;
    const int TOTAL_READINGS   = 110;  // 10 discard + 100 used

    int validCount = 0;

    for (int i = 0; i < TOTAL_READINGS; i++) {
        if (baro.performReading()) {
            float alt = baro.readAltitude(1013.25);

            // Skip first few valid readings
            if (validCount >= DISCARD_READINGS) {
                altSum += alt;
                altSamples++;
            }

            validCount++;
        }
        delay(10);
    }  

    if (altSamples > 0) {
        groundAltitude = altSum / altSamples;

        DEBUG_PRINT("Ground altitude: ");
        DEBUG_PRINT2(groundAltitude, 2);
        DEBUG_PRINTLN(" m");

        logDebugMessage("Barometer tare complete!");
        logDebugMessage("Ground altitude: ", groundAltitude, 2);
    } else {
        DEBUG_PRINTLN("ERROR: Could not read ground altitude!");
        logDebugMessage("ERROR: Could not read ground altitude!");
    }
}

// ============================================================================
// READ BAROMETER - Just read it when called
// ============================================================================

void readBarometer(State &currentState) {
    static unsigned long lastRead = 0;
    unsigned long now = millis();
    
    // Rate limit to 100 Hz (10ms) to avoid overwhelming I2C
    if (now - lastRead < 10) return;
    lastRead = now;
    
    if (!baro.performReading()) {
        DEBUG_PRINTLN("Baro read failed");
        return;
    }
    
    // Just like Python: bmp.altitude, bmp.pressure, bmp.temperature
    currentState.baroAltitude    = baro.readAltitude(1013.25f);
    currentState.baroPressure    = baro.pressure / 100.0f;
    currentState.baroTemperature = baro.temperature;
}
