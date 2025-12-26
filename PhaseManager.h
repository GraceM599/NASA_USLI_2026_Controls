/*
 * ============================================================================
 * PHASE MANAGER - Header File
 * ============================================================================
 * Handles flight phase detection and state machine transitions
 */

#ifndef PHASE_MANAGER_H
#define PHASE_MANAGER_H

#include <Arduino.h>
#include "State.h"
#include "KalmanFilter.h"

// ============================================================================
// FLIGHT PHASES
// ============================================================================

enum FlightPhase {
    GROUND_IDLE,
    ARMED,
    POWERED_ASCENT,
    COASTING,        // Motor burned out, airbrakes active
    DESCENT,         // Past apogee, airbrakes retracted
    LANDED
};

extern const char* phaseNames[];

// ============================================================================
// PHASE DETECTION CONFIGURATION
// ============================================================================

#define PHASE_BUFFER_SIZE 50        // 0.5 seconds at 100 Hz

// Liftoff detection (ARMED → POWERED_ASCENT)
#define LIFTOFF_ACCEL_THRESHOLD 2.0f      // m/s² - adjust from motor specs
#define LIFTOFF_ALT_THRESHOLD 0.6f        // meters AGL

// Burnout detection (POWERED_ASCENT → COASTING)
#define BURNOUT_TIME_MIN 1500000          // 1.5 seconds (microseconds) for testing

// Apogee detection (COASTING → DESCENT)
#define APOGEE_SAMPLES_CHECK 10           // Check last 10 samples
#define APOGEE_ALT_DECREASE_THRESHOLD 0.2f // Altitude must decrease by at least 0.3m
#define APOGEE_VELOCITY_THRESHOLD -1.0f   // Velocity must be consistently negative

// Landing detection (DESCENT → LANDED)
#define LANDING_ALT_THRESHOLD 10.0f        // meters AGL (generous threshold)
#define LANDING_SAMPLES_CHECK 50          // Check last 20 samples (0.2 seconds at 100 Hz)
#define LANDING_ALT_CHANGE_MAX 0.03f        // Altitude change < 0.5m over samples
#define LANDING_VEL_CHANGE_MAX 0.1f        // Velocity change < 0.5m/s over samples

// ============================================================================
// DATA STRUCTURES
// ============================================================================

// Phase detection data (for analysis only)
struct PhaseDetectionData {
    float altitude;
    float velocity;
    float accelMagnitude;
    unsigned long timestamp;
};

// ============================================================================
// EXTERNAL VARIABLES (declared in PhaseManager.cpp)
// ============================================================================

extern FlightPhase currentPhase;
extern unsigned long phaseStartTime;
extern unsigned long liftoffTime;
extern float maxAltitude;
extern unsigned long lastMovementTime;

#ifdef ENABLE_PHASE_BUFFER
extern PhaseDetectionData phaseBuffer[PHASE_BUFFER_SIZE];
extern int phaseBufferIndex;
extern bool phaseBufferFilled;
#endif

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

// Initialization
void initializePhaseManager();

// Main phase update (call at 100 Hz from control loop)
void updateFlightPhase(State& currentState, float groundAltitude, KalmanFilter& KF);

// Control check
bool shouldControl();

// Phase buffer functions (if enabled)
#ifdef ENABLE_PHASE_BUFFER
void addToPhaseBuffer(float altitude, float velocity, float accelMag);
PhaseDetectionData getPhaseDataFromPast(int samplesAgo);
bool isAltitudeDecreasingFor(int numSamples);
bool isVelocityNegativeFor(int numSamples);
float getAltitudeChange(int numSamples);
float getAverageAcceleration(int numSamples);
#endif

#endif // PHASE_MANAGER_H
