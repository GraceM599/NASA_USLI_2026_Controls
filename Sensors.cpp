/*
 * ============================================================================
 * SENSORS - Implementation File
 * ============================================================================
 * Handles all sensor initialization, reading, and calibration
 */

#include "Sensors.h"
#include "Debug.h"
#include "Logger.h"

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

MTi *imu = NULL;
Adafruit_BMP3XX baro;

float imuTare[6] = {0.0f};
bool imuTared = false;

float groundAltitude = 0.0f;

String nmeaBuffer = "";
unsigned long lastGPSRead = 0;

// ============================================================================
// IMU FUNCTIONS
// ============================================================================

bool initializeIMU() {
    DEBUG_PRINTLN("Initializing IMU...");
    logDebugMessage("Initializing IMU...");

    pinMode(IMU_DRDY_PIN, INPUT);
    imu = new MTi(IMU_ADDRESS, IMU_DRDY_PIN);
    
    if (!imu->detect(1000)) {
        DEBUG_PRINTLN("ERROR: IMU not detected! Check connections.");
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

    // Give IMU time to start producing data
    delay(500);
    
    DEBUG_PRINTLN("IMU initialized!");
    logDebugMessage("IMU initialized!");
    
    return true;
}

void performIMUTare() {
    DEBUG_PRINTLN("Performing IMU tare (bias removal)...");
    DEBUG_PRINTLN("Keep sensor stationary for 3 seconds...");
    logDebugMessage("Performing IMU tare...");
    
    int samples = 0;
    unsigned long tareStart = millis();
    float sum[6] = {0};
    
    while (millis() - tareStart < 3000) {
        if (digitalRead(IMU_DRDY_PIN)) {  // Check DRDY first!
            imu->readMessages();
            float* acc = imu->getAcceleration();
            float* gyro = imu->getRateOfTurn();
            
            // Only add if valid (not NaN or extreme values)
            if (abs(acc[0]) < 50 && abs(acc[1]) < 50 && abs(acc[2]) < 50) {
                sum[0] += acc[0];
                sum[1] += acc[1];
                sum[2] += acc[2];
                sum[3] += gyro[0];
                sum[4] += gyro[1];
                sum[5] += gyro[2];
                samples++;
            }
        }
        delay(10);  // Wait for next data ready
    }
    
    if (samples > 0) {
        for (int i = 0; i < 6; i++) {
            imuTare[i] = sum[i] / samples;
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

void readIMU(State& currentState) {
    if (imu && digitalRead(IMU_DRDY_PIN)) {
        imu->readMessages();
        
        float* acc = imu->getAcceleration();
        float* gyro = imu->getRateOfTurn();
        
        // Apply tare (bias removal)
        if (imuTared) {
            currentState.accelX = acc[0] - imuTare[0];
            currentState.accelY = acc[1] - imuTare[1];
            currentState.accelZ = acc[2] - imuTare[2];
            currentState.gyroX = gyro[0] - imuTare[3];
            currentState.gyroY = gyro[1] - imuTare[4];
            currentState.gyroZ = gyro[2] - imuTare[5];
        } else {
            currentState.accelX = acc[0];
            currentState.accelY = acc[1];
            currentState.accelZ = acc[2];
            currentState.gyroX = gyro[0];
            currentState.gyroY = gyro[1];
            currentState.gyroZ = gyro[2];
        }
        
        VERBOSE_PRINT("IMU: ax=");
        VERBOSE_PRINT(currentState.accelX);
        VERBOSE_PRINT(" ay=");
        VERBOSE_PRINT(currentState.accelY);
        VERBOSE_PRINT(" az=");
        VERBOSE_PRINTLN(currentState.accelZ);
    }
}

// ============================================================================
// BAROMETER FUNCTIONS
// ============================================================================

bool initializeBarometer() {
    DEBUG_PRINTLN("Initializing Barometer...");
    logDebugMessage("Initializing Barometer...");
    
    if (!baro.begin_I2C()) {
        DEBUG_PRINTLN("ERROR: BMP390 not detected! Check connections.");
        return false;
    }
    
    // Configure BMP390 for high performance
    baro.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    baro.setPressureOversampling(BMP3_OVERSAMPLING_32X);
    baro.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    baro.setOutputDataRate(BMP3_ODR_100_HZ);
    
    DEBUG_PRINTLN("Barometer initialized!");
    logDebugMessage("Barometer initialized!");
    
    return true;
}

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

void readBarometer(State& currentState) {
    if (!baro.performReading()) {
        DEBUG_PRINTLN("Failed to read barometer!");
        return;
    }
    
    currentState.baroAltitude = baro.readAltitude(1013.25);  // Sea level pressure
    currentState.baroPressure = baro.pressure / 100.0f;      // Convert Pa to hPa
    currentState.baroTemperature = baro.temperature;
    
    VERBOSE_PRINT("Baro: ");
    VERBOSE_PRINT(currentState.baroAltitude);
    VERBOSE_PRINTLN(" m");
}

// ============================================================================
// GPS FUNCTIONS
// ============================================================================

bool initializeGPS() {
    DEBUG_PRINTLN("Initializing GPS...");
    logDebugMessage("Initializing GPS...");
    
    GPS_SERIAL.begin(GPS_BAUD_INITIAL);
    delay(200);
    GPS_SERIAL.print(PMTK_SET_BAUD_115200);
    delay(200);
    GPS_SERIAL.end();
    GPS_SERIAL.begin(GPS_BAUD_OPERATING);
    delay(200);
    GPS_SERIAL.print(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    delay(100);
    GPS_SERIAL.print(PMTK_SET_NMEA_UPDATE_10HZ);
    delay(100);
    
    DEBUG_PRINTLN("GPS initialized!");
    logDebugMessage("GPS initialized!");
    
    return true;
}

void parseNMEA(String sentence, State& currentState) {
    // Parse GPGGA sentence (position and fix data)
    if (sentence.startsWith("$GPGGA") || sentence.startsWith("$GNGGA")) {
    int commaPos[14];
    int commaCount = 0;

    for (int i = 0; i < sentence.length() && commaCount < 14; i++) {
        if (sentence.charAt(i) == ',') {
            commaPos[commaCount++] = i;
        }
    }

    if (commaCount >= 9) {
        String latStr = sentence.substring(commaPos[1] + 1, commaPos[2]);
        String latDir = sentence.substring(commaPos[2] + 1, commaPos[3]);

        String lonStr = sentence.substring(commaPos[3] + 1, commaPos[4]);
        String lonDir = sentence.substring(commaPos[4] + 1, commaPos[5]);

        String fixQuality = sentence.substring(commaPos[5] + 1, commaPos[6]);
        String satStr     = sentence.substring(commaPos[6] + 1, commaPos[7]);
        String altStr     = sentence.substring(commaPos[8] + 1, commaPos[9]);

        // ─────────────────────────────────────────────
        // FIX VALIDITY CHECK (PUT THIS HERE)
        // ─────────────────────────────────────────────
        int fix = fixQuality.toInt();
        currentState.gpsHasFix = (fix > 0);

        if (!currentState.gpsHasFix) {
        currentState.gpsLat        = 0.0f;
        currentState.gpsLon        = 0.0f;
        currentState.gpsAltitude   = 0.0f;
        currentState.gpsSpeed      = 0.0f;
        currentState.gpsSatellites = 0;
        return;
}

        // ─────────────────────────────────────────────
        // SAFE TO PARSE POSITION NOW
        // ─────────────────────────────────────────────

        // Latitude
        if (latStr.length() > 0) {
            float lat = latStr.toFloat();
            int degrees = (int)(lat / 100);
            float minutes = lat - (degrees * 100);
            currentState.gpsLat = degrees + (minutes / 60.0f);
            if (latDir == "S") currentState.gpsLat = -currentState.gpsLat;
        }

        // Longitude
        if (lonStr.length() > 0) {
            float lon = lonStr.toFloat();
            int degrees = (int)(lon / 100);
            float minutes = lon - (degrees * 100);
            currentState.gpsLon = degrees + (minutes / 60.0f);
            if (lonDir == "W") currentState.gpsLon = -currentState.gpsLon;
        }

        // Altitude
        if (altStr.length() > 0) {
            currentState.gpsAltitude = altStr.toFloat();
        }

        // Satellites
        if (satStr.length() > 0) {
            currentState.gpsSatellites = satStr.toInt();
        }
    }
}

    
    // Parse GPRMC sentence (speed data)
    else if (sentence.startsWith("$GPRMC") || sentence.startsWith("$GNRMC")) {
    int commaPos[12];
    int commaCount = 0;

    for (int i = 0; i < sentence.length() && commaCount < 12; i++) {
        if (sentence.charAt(i) == ',') {
            commaPos[commaCount++] = i;
        }
    }

    if (commaCount >= 7) {
        String status   = sentence.substring(commaPos[1] + 1, commaPos[2]);
        String speedStr = sentence.substring(commaPos[6] + 1, commaPos[7]);

        // ❗ Reject invalid RMC
        if (status != "A") {
            currentState.gpsSpeed = 0.0f;
            return;
        }

        // ❗ Also require GGA fix
        if (!currentState.gpsHasFix) {
            currentState.gpsSpeed = 0.0f;
            return;
        }

        if (speedStr.length() > 0) {
            float speedKnots = speedStr.toFloat();
            currentState.gpsSpeed = speedKnots * 0.514444f;
        }
    }
}

}

void readGPS(State& currentState) {
    unsigned long now = millis();
    
    // Read GPS data at ~10 Hz
    if (now - lastGPSRead >= 100) {
        lastGPSRead = now;
        
        while (GPS_SERIAL.available()) {
            char c = GPS_SERIAL.read();
            
            if (c == '\n') {
                // Complete sentence received
                if (nmeaBuffer.length() > 0) {
                    parseNMEA(nmeaBuffer, currentState);
                    nmeaBuffer = "";
                }
            } else if (c != '\r') {
                nmeaBuffer += c;
            }
            
            // Prevent buffer overflow
            if (nmeaBuffer.length() > 120) {
                nmeaBuffer = "";
            }
        }
    }
}
