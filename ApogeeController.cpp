#include "ApogeeController.h"

// ============================================================================
// DRAG COEFFICIENT TABLE    // Change with real data later
// ============================================================================

// Flap drag coefficient vs angle (0-45 degrees in 5-degree increments)
// 45 was just chosen arbitrarily
const float ApogeeController::FLAP_ANGLES[NUM_FLAP_ANGLES] = {
    0, 5, 10, 15, 20, 25, 30, 35, 40, 45
};

// Change with real data later
const float ApogeeController::FLAP_CD_VALUES[NUM_FLAP_ANGLES] = {
    0.00,   // 0 degrees 
    0.05,   // 5 degrees
    0.12,   // 10 degrees  
    0.20,   // 15 degrees
    0.30,   // 20 degrees
    0.42,   // 25 degrees
    0.56,   // 30 degrees
    0.72,   // 35 degrees
    0.90,   // 40 degrees
    1.10    // 45 degrees 
};

// ============================================================================
// CONSTRUCTOR
// ============================================================================

ApogeeController::ApogeeController() {
    previousError = 0.0f;
    integralError = 0.0f;
    previousApogee = 0.0f;
    currentFlapAngle = 0.0f;
    lastUpdateTime = 0;
    controlActive = false;
    predictedApogee = 0.0f;
    simulationSteps = 0;
}

// ============================================================================
// INITIALIZE CONTROLLER
// ============================================================================

void ApogeeController::initialize() {
    previousError = 0.0f;
    integralError = 0.0f;
    previousApogee = 0.0f;
    currentFlapAngle = 0.0f;
    lastUpdateTime = micros();
    controlActive = false;
    predictedApogee = 0.0f;
}

// ============================================================================
// CHECK IF CONTROL SHOULD BE ACTIVE
// ============================================================================

bool ApogeeController::shouldControl(FlightPhase phase) {
    // Only control during coast phase (after burnout, before apogee)
    return (phase == COASTING);
}

// ============================================================================
// LOOKUP FLAP DRAG COEFFICIENT FROM TABLE
// ============================================================================

float ApogeeController::getFlapDragCoefficient(float angle) {
    // Clamp angle to valid range
    if (angle <= FLAP_ANGLES[0]) return FLAP_CD_VALUES[0];
    if (angle >= FLAP_ANGLES[NUM_FLAP_ANGLES - 1]) return FLAP_CD_VALUES[NUM_FLAP_ANGLES - 1];
    
    // Linear interpolation between table values
    // IRL might not be linear, change later once we get the real data
    for (int i = 0; i < NUM_FLAP_ANGLES - 1; i++) {
        if (angle >= FLAP_ANGLES[i] && angle <= FLAP_ANGLES[i + 1]) {
            float t = (angle - FLAP_ANGLES[i]) / (FLAP_ANGLES[i + 1] - FLAP_ANGLES[i]);
            return FLAP_CD_VALUES[i] + t * (FLAP_CD_VALUES[i + 1] - FLAP_CD_VALUES[i]);
        }
    }
    
    return FLAP_CD_VALUES[0];  // Fallback
}

// ============================================================================
// SIMULATE TRAJECTORY TO PREDICT APOGEE (EULER INTEGRATION) (Can do RK4 but idk if we would wanna make it 4 times slower)
// ============================================================================

float ApogeeController::predictApogee(float currentAltitude, float currentVelocity, float flapAngle) {
    float altitude = currentAltitude;
    float velocity = currentVelocity;
    float time = 0.0f;
    simulationSteps = 0;
    
    // Get total drag coefficient (rocket + flaps)
    float flapCd = getFlapDragCoefficient(flapAngle);
    float totalCd = ROCKET_CD + flapCd;
    
    // Simulate until velocity becomes negative (past apogee) or timeout
    while (velocity > 0.0f && time < MAX_SIM_TIME) {
        // Calculate drag force: F_drag = 0.5 * rho * v^2 * Cd * A
        float dragForce = 0.5f * AIR_DENSITY * velocity * velocity * totalCd * REFERENCE_AREA;
        
        // Total acceleration: a = -g - (F_drag / m)
        float acceleration = -GRAVITY - (dragForce / ROCKET_MASS);
        
        // Euler integration
        velocity += acceleration * SIM_TIME_STEP;
        altitude += velocity * SIM_TIME_STEP;
        
        time += SIM_TIME_STEP;
        simulationSteps++;
        
        // Safety check
        if (simulationSteps > 1000) break;  // Prevent infinite loop
    }
    
    return altitude;  // Return predicted apogee
}

// ============================================================================
// PID CONTROLLER - COMPUTE NEW FLAP ANGLE
// ============================================================================

float ApogeeController::computeControl(float predictedApogee, float dt) {
    // Calculate error (negative = undershoot, positive = overshoot)
    // If predicted > target: we're going too high, need MORE drag (open flaps)
    // If predicted < target: we're going too low, need LESS drag (close flaps)
    float error = predictedApogee - TARGET_APOGEE;
    
    // Proportional term: directly maps error to flap angle
    float pTerm = KP * error;
    
    // Integral term 
    // Accumulates error over time for steady-state correction
    integralError += error * dt;
    float iTerm = KI * integralError;
    
    // Derivative term: responds to RATE OF CHANGE of error
    // If error is increasing rapidly, add more drag
    // If error is decreasing, we're correcting, ease off
    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = (error - previousError) / dt;
    }
    float dTerm = KD * derivative;
    
    // PID output: desired flap angle based on error
    // This is the ABSOLUTE angle we want, not an increment
    float desiredAngle = pTerm + iTerm + dTerm;
    
    // Update previous values for next iteration
    previousError = error;
    previousApogee = predictedApogee;
    
    // Clamp to valid range
    if (desiredAngle > MAX_FLAP_ANGLE) desiredAngle = MAX_FLAP_ANGLE;
    if (desiredAngle < MIN_FLAP_ANGLE) desiredAngle = MIN_FLAP_ANGLE;
    
    return desiredAngle;
}

// ============================================================================
// MAIN UPDATE FUNCTION 
// ============================================================================

float ApogeeController::update(State& currentState, FlightPhase phase) {
    unsigned long now = micros();
    
    // Check if control should be active
    if (!shouldControl(phase)) {
        controlActive = false;
        currentFlapAngle = 0.0f;  // Retract flaps
        return currentFlapAngle;
    }
    
    controlActive = true;
    
    // Check if it's time to update (every 1 second)
    if (now - lastUpdateTime < CONTROL_UPDATE_INTERVAL) {
        return currentFlapAngle;  // Return current angle, no update
    }
    
    float dt = (now - lastUpdateTime) / 1000000.0f;  // Convert to seconds
    lastUpdateTime = now;
    
    // Step 1: Predict apogee with current flap angle
    predictedApogee = predictApogee(
        currentState.estimatedAltitude,
        currentState.estimatedVelocity,
        currentFlapAngle
    );
    
    // Step 2: Compute new flap angle using PID
    currentFlapAngle = computeControl(predictedApogee, dt);
    
    return currentFlapAngle;
}

// ============================================================================
// GETTERS FOR TELEMETRY/DEBUGGING
// ============================================================================

float ApogeeController::getPredictedApogee() {
    return predictedApogee;
}

float ApogeeController::getCurrentFlapAngle() {
    return currentFlapAngle;
}

float ApogeeController::getApogeeError() {
    return predictedApogee - TARGET_APOGEE;
}

int ApogeeController::getSimulationSteps() {
    return simulationSteps;
}

bool ApogeeController::isControlActive() {
    return controlActive;
}
