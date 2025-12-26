#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() : is_initialized(false), last_micros(0) {
    // Initial state and covariance (same as your Pi code)
    x << 0, 0;
    P << 1, 0,
         0, 1;

    // Q: Process noise covariance
    Q << 0.5, 0,
         0, 1.5;

    // R: Measurement noise (Trust measurement less -> smoother velocity)
    R = 60.0f;
}

void KalmanFilter::initialize(float initial_alt) {
    x(0) = initial_alt;
    x(1) = 0;
    last_micros = micros();
    is_initialized = true;
}

void KalmanFilter::update(float z, uint32_t current_micros) {
    if (!is_initialized) {
        initialize(z);
        return;
    }

    // --- Dynamic dt Calculation with Safety ---
    float dt = (current_micros - last_micros) * 1.0e-6f;
    last_micros = current_micros;

    // Guard against first-loop spikes or zero-time errors
    // If dt is unrealistic, fallback to 0.031s
    if (dt <= 0.0f || dt > 0.5f) {
        dt = 0.031f;
    }

    // --- 1. Predict Step ---
    // A: State transition matrix
    Eigen::Matrix2f A;
    A << 1, dt,
         0, 1;

    // x_p = A * x
    Eigen::Vector2f xp = A * x;

    // P_p = A * P * A.transpose() + Q
    Eigen::Matrix2f Pp = A * P * A.transpose() + Q;

    // --- 2. Update (Correct) Step ---
    // H: Measurement matrix [1, 0]
    Eigen::RowVector2f H(1, 0);

    // Innovation covariance S = H * Pp * H' + R
    // Note: Since H is [1, 0], H * Pp * H.transpose() is just Pp(0,0)
    float S = Pp(0, 0) + R;

    // Kalman Gain K = Pp * H' / S
    Eigen::Vector2f K = Pp.col(0) / S;

    // Innovation (Measurement residual)
    float innovation = z - xp(0);

    // New State: x = xp + K * innovation
    x = xp + K * innovation;

    // New Covariance: P = (I - K*H) * Pp
    P = (Eigen::Matrix2f::Identity() - K * H) * Pp;
}
