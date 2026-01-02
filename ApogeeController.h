/*
 * ============================================================================
 * APOGEE CONTROLLER - Header File
 * ============================================================================
 * 
 */

#ifndef APOGEE_CONTROLLER_H
#define APOGEE_CONTROLLER_H

#include <Arduino.h>
#include "State.h"
#include "PhaseManager.h"

// ============================================================================
// CONFIGURATION
// ============================================================================

// Target apogee (meters AGL)
#define TARGET_APOGEE 10.0f  // ~4250 feet --> make sure convert to m

// Physical constants
#define GRAVITY 9.81f  // m/s^2

// Rocket parameters (UPDATE THESE WITH YOUR VALUES)
#define ROCKET_MASS 0.1f           // kg (dry mass after burnout)
#define ROCKET_DIAMETER 0.1524f     // meters (6 inches)
#define ROCKET_CD 0.05f             // Baseline drag coefficient            // need to get data from ROhit
#define REFERENCE_AREA 0.01824f     // m^2 (pi * (diameter/2)^2)
#define AIR_DENSITY 1.225f          // kg/m^3 at sea level

// Control parameters
#define CONTROL_UPDATE_INTERVAL 500000  // microseconds (0.5 second)
#define MAX_FLAP_ANGLE 45.0f        // degrees
#define MIN_FLAP_ANGLE 0.0f         // degrees (fully retracted)

// PID gains (TUNE THESE!)
#define KP 2.0f   // Proportional gain
#define KI 0.0f   // Integral gain 
#define KD 0.5f   // Derivative gain

// Simulation parameters
#define SIM_TIME_STEP 0.1f          // seconds (Euler integration step)
#define MAX_SIM_TIME 30.0f          // seconds (max simulation duration)

// Drag coefficient table size
#define NUM_FLAP_ANGLES 10

// ============================================================================
// CONTROLLER CLASS
// ============================================================================

class ApogeeController {
private:
    // PID state
    float previousError;
    float integralError;
    float previousApogee;
    
    // Control state
    float currentFlapAngle;
    unsigned long lastUpdateTime;
    bool controlActive;
    
    // Statistics
    float predictedApogee;
    int simulationSteps;
    
    // Drag coefficient lookup table
    static const float FLAP_ANGLES[NUM_FLAP_ANGLES];
    static const float FLAP_CD_VALUES[NUM_FLAP_ANGLES];
    
    // Private methods
    float getFlapDragCoefficient(float angle);
    float computeControl(float predictedApogee, float dt);
    
public:
    ApogeeController();
    
    void initialize();
    bool shouldControl(FlightPhase phase);
    float update(State& currentState, FlightPhase phase);
    
    // Prediction function - public for testing
    float predictApogee(float currentAltitude, float currentVelocity, float flapAngle);
    
    // Getters for telemetry/debugging
    float getPredictedApogee();
    float getCurrentFlapAngle();
    float getApogeeError();
    int getSimulationSteps();
    bool isControlActive();
};

#endif  // APOGEE_CONTROLLER_H
