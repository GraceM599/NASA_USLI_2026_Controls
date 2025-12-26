#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <ArduinoEigenDense.h>

class KalmanFilter {
public:
    KalmanFilter();
    
    // Safety-hardened update function
    void update(float z_measured, uint32_t current_micros);
    
    float getAltitude() const { return x(0); }
    float getVelocity() const { return x(1); }
    
    // Call this in setup() to set the starting ground altitude
    void initialize(float initial_alt);

private:
    Eigen::Vector2f x;        // State: [altitude, velocity]
    Eigen::Matrix2f P;        // Covariance
    Eigen::Matrix2f Q;        // Process Noise
    float R;                  // Measurement Noise
    
    uint32_t last_micros;     // For dynamic dt calculation
    bool is_initialized;
};

#endif
