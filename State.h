#ifndef STATE_H
#define STATE_H

// Current state (holds latest sensor readings)
struct State {
    // IMU
    float accelX, accelY, accelZ;  // m/s²
    float gyroX, gyroY, gyroZ;     // rad/s
    float roll, pitch, yaw;
    
    // Barometer
    float baroAltitude;            // meters
    float baroPressure;            // hPa
    float baroTemperature;         // °C
    
    // GPS
    float gpsLat;
    float gpsLon;
    float gpsAltitude;
    float gpsSpeed;
    uint8_t gpsSatellites;
    bool gpsHasFix;
    
    // TODO: Kalman estimates
    float estimatedAltitude;    // meters 
    float estimatedVelocity;    // m/s
};

#endif
