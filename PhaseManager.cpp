/*
 * ============================================================================
 * PHASE MANAGER - Implementation File
 * ============================================================================
 * Handles flight phase detection and state machine transitions
 */

#include "PhaseManager.h"
#include "Debug.h"
#include "Logger.h"

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

const char* phaseNames[] = {
    "GROUND_IDLE",
    "ARMED",
    "POWERED_ASCENT",
    "COASTING",
    "DESCENT",
    "LANDED"
};

FlightPhase currentPhase = GROUND_IDLE;
unsigned long phaseStartTime = 0;
unsigned long liftoffTime = 0;
float maxAltitude = 0.0f;
unsigned long lastMovementTime = 0;

#ifdef ENABLE_PHASE_BUFFER
PhaseDetectionData phaseBuffer[PHASE_BUFFER_SIZE];
int phaseBufferIndex = 0;
bool phaseBufferFilled = false;
#endif

// ============================================================================
// INITIALIZATION
// ============================================================================

void initializePhaseManager() {
    currentPhase = GROUND_IDLE;
    phaseStartTime = 0;
    liftoffTime = 0;
    maxAltitude = 0.0f;
    lastMovementTime = 0;
    
    #ifdef ENABLE_PHASE_BUFFER
    phaseBufferIndex = 0;
    phaseBufferFilled = false;
    #endif
}

// ============================================================================
// PHASE DETECTION BUFFER FUNCTIONS
// ============================================================================

#ifdef ENABLE_PHASE_BUFFER

void addToPhaseBuffer(float altitude, float velocity, float accelMag) {
    phaseBuffer[phaseBufferIndex].altitude = altitude;
    phaseBuffer[phaseBufferIndex].velocity = velocity;
    phaseBuffer[phaseBufferIndex].accelMagnitude = accelMag;
    phaseBuffer[phaseBufferIndex].timestamp = micros();
    
    phaseBufferIndex = (phaseBufferIndex + 1) % PHASE_BUFFER_SIZE;
    
    if (phaseBufferIndex == 0) {
        phaseBufferFilled = true;
    }
}

PhaseDetectionData getPhaseDataFromPast(int samplesAgo) {
    if (samplesAgo < 0 || samplesAgo >= PHASE_BUFFER_SIZE) {
        return phaseBuffer[(phaseBufferIndex - 1 + PHASE_BUFFER_SIZE) % PHASE_BUFFER_SIZE];
    }
    
    int index = (phaseBufferIndex - 1 - samplesAgo + PHASE_BUFFER_SIZE) % PHASE_BUFFER_SIZE;
    return phaseBuffer[index];
}

bool isAltitudeDecreasingFor(int numSamples) {
    if (!phaseBufferFilled && phaseBufferIndex < numSamples) {
        return false;
    }
    
    for (int i = 1; i < numSamples; i++) {
        PhaseDetectionData current = getPhaseDataFromPast(i - 1);
        PhaseDetectionData previous = getPhaseDataFromPast(i);
        
        if (current.altitude >= previous.altitude) {
            return false;
        }
    }
    
    return true;
}

bool isVelocityNegativeFor(int numSamples) {
    if (!phaseBufferFilled && phaseBufferIndex < numSamples) {
        return false;
    }
    
    for (int i = 0; i < numSamples; i++) {
        PhaseDetectionData data = getPhaseDataFromPast(i);
        if (data.velocity >= 0.0f) {
            return false;
        }
    }
    
    return true;
}

float getAltitudeChange(int numSamples) {
    if (!phaseBufferFilled && phaseBufferIndex < numSamples) {
        return 0.0f;
    }
    
    PhaseDetectionData current = getPhaseDataFromPast(0);
    PhaseDetectionData past = getPhaseDataFromPast(numSamples - 1);
    
    return current.altitude - past.altitude;
}

float getVelocityChange(int numSamples) {
    if (!phaseBufferFilled && phaseBufferIndex < numSamples) {
        return 0.0f;
    }
    
    PhaseDetectionData current = getPhaseDataFromPast(0);
    PhaseDetectionData past = getPhaseDataFromPast(numSamples - 1);
    
    return abs(current.velocity - past.velocity);
}

bool isStableFor(int numSamples, float maxAltChange, float maxVelChange) {
    if (!phaseBufferFilled && phaseBufferIndex < numSamples) {
        return false;  // Not enough data yet
    }
    
    // Get min and max altitude over the samples
    float minAlt = getPhaseDataFromPast(0).altitude;
    float maxAlt = minAlt;
    float minVel = getPhaseDataFromPast(0).velocity;
    float maxVel = minVel;
    
    for (int i = 1; i < numSamples; i++) {
        PhaseDetectionData data = getPhaseDataFromPast(i);
        
        if (data.altitude < minAlt) minAlt = data.altitude;
        if (data.altitude > maxAlt) maxAlt = data.altitude;
        if (data.velocity < minVel) minVel = data.velocity;
        if (data.velocity > maxVel) maxVel = data.velocity;
    }
    
    float altRange = maxAlt - minAlt;
    float velRange = maxVel - minVel;
    
    // Both altitude and velocity must be stable
    return (altRange < maxAltChange) && (velRange < maxVelChange);
}

float getAverageAcceleration(int numSamples) {
    if (!phaseBufferFilled && phaseBufferIndex < numSamples) {
        numSamples = phaseBufferIndex;
    }
    
    if (numSamples == 0) return 0.0f;
    
    float sum = 0.0f;
    for (int i = 0; i < numSamples; i++) {
        sum += getPhaseDataFromPast(i).accelMagnitude;
    }
    
    return sum / numSamples;
}

#endif  // ENABLE_PHASE_BUFFER

// ============================================================================
// FLIGHT PHASE MANAGER
// ============================================================================

void updateFlightPhase(State& currentState, float groundAltitude, KalmanFilter& KF) {
    unsigned long now = micros();
    
    // Calculate derived values
    float AGL = currentState.baroAltitude - groundAltitude;

    // Kalman update
    KF.update(AGL, now);
    currentState.estimatedAltitude = KF.getAltitude();
    currentState.estimatedVelocity = KF.getVelocity();

    float accelMagnitude = sqrt(currentState.accelX * currentState.accelX +
                                currentState.accelY * currentState.accelY +
                                currentState.accelZ * currentState.accelZ);
    
    #ifdef ENABLE_PHASE_BUFFER
    addToPhaseBuffer(currentState.estimatedAltitude, currentState.estimatedVelocity, accelMagnitude);
    #endif
    
    // Track max altitude
    if (currentState.estimatedAltitude > maxAltitude) {
        maxAltitude = currentState.estimatedAltitude;
    }
    
    // ========================================================================
    // STATE MACHINE
    // ========================================================================
    
    switch(currentPhase) {
        case GROUND_IDLE:
            // Should never be here (auto-armed in setup)
            break;
            
        case ARMED: {
            // ================================================================
            // LIFTOFF DETECTION: ARMED → POWERED_ASCENT
            // Require BOTH high acceleration AND altitude gain
            // ================================================================
            bool highAcceleration = (accelMagnitude > LIFTOFF_ACCEL_THRESHOLD);
            bool altitudeGain = (currentState.estimatedAltitude > LIFTOFF_ALT_THRESHOLD);
            
            if (highAcceleration && altitudeGain) {
                currentPhase = POWERED_ASCENT;
                liftoffTime = now;
                phaseStartTime = now;
                
                #ifdef DEBUG_PHASE_TRANSITIONS
                DEBUG_PRINTLN(">>> LIFTOFF DETECTED <<<");
                DEBUG_PRINT("Accel: "); DEBUG_PRINT2(accelMagnitude, 2);
                DEBUG_PRINT(" m/s² | Alt: "); DEBUG_PRINT2(currentState.estimatedAltitude, 2);
                DEBUG_PRINTLN(" m");
                logDebugMessage(">>> LIFTOFF DETECTED <<<");
                #endif
            }
        }
            break;
            
        case POWERED_ASCENT: {
            // ================================================================
            // BURNOUT DETECTION: POWERED_ASCENT → COASTING
            // Motor burn time
            // ================================================================
            bool minBurnTime = ((now - liftoffTime) > BURNOUT_TIME_MIN);
            
            if (minBurnTime) {
                currentPhase = COASTING;
                phaseStartTime = now;
                
                #ifdef DEBUG_PHASE_TRANSITIONS
                DEBUG_PRINTLN(">>> MOTOR BURNOUT - COASTING <<<");
                DEBUG_PRINT("Velocity: "); DEBUG_PRINT2(currentState.estimatedVelocity, 2);
                DEBUG_PRINT(" m/s | Alt: "); DEBUG_PRINT2(currentState.estimatedAltitude, 2);
                DEBUG_PRINTLN(" m");
                logDebugMessage(">>> MOTOR BURNOUT - COASTING <<<");
                #endif
            }
        }
            break;
            
        case COASTING: {
            // ================================================================
            // APOGEE DETECTION: COASTING → DESCENT
            // Robust detection using multiple criteria:
            // 1. Velocity consistently negative for N samples
            // 2. Altitude has decreased over the last N samples
            // 3. Total altitude drop exceeds threshold
            // ================================================================
            
            #ifdef ENABLE_PHASE_BUFFER
            bool velocityNegative = isVelocityNegativeFor(APOGEE_SAMPLES_CHECK);
            bool altitudeDecreasing = isAltitudeDecreasingFor(APOGEE_SAMPLES_CHECK);
            float altChange = getAltitudeChange(APOGEE_SAMPLES_CHECK);
            bool significantDrop = (altChange < -APOGEE_ALT_DECREASE_THRESHOLD);
            
            // All three conditions must be met for robust apogee detection
            if (velocityNegative && altitudeDecreasing && significantDrop) {
                currentPhase = DESCENT;
                phaseStartTime = now;
                
                #ifdef DEBUG_PHASE_TRANSITIONS
                DEBUG_PRINTLN(">>> APOGEE DETECTED - DESCENDING <<<");
                DEBUG_PRINT("Max Alt: "); DEBUG_PRINT2(maxAltitude, 2);
                DEBUG_PRINT(" m | Velocity: "); DEBUG_PRINT2(currentState.estimatedVelocity, 2);
                DEBUG_PRINT(" m/s | Alt Drop: "); DEBUG_PRINT2(-altChange, 2);
                DEBUG_PRINTLN(" m");
                logDebugMessage(">>> APOGEE DETECTED - DESCENDING <<<");
                logDebugMessage("Max Altitude: ", maxAltitude, 2);
                logDebugMessage("Velocity: ", currentState.estimatedVelocity, 2);
                logDebugMessage("Altitude Drop: ", -altChange, 2);
                #endif
            }
            #else
            // Fallback: simple velocity check (less robust)
            bool descendingVelocity = (currentState.estimatedVelocity < APOGEE_VELOCITY_THRESHOLD);
            
            if (descendingVelocity) {
                currentPhase = DESCENT;
                phaseStartTime = now;
                
                #ifdef DEBUG_PHASE_TRANSITIONS
                DEBUG_PRINTLN(">>> APOGEE DETECTED - DESCENDING <<<");
                DEBUG_PRINT("Max Alt: "); DEBUG_PRINT2(maxAltitude, 2);
                DEBUG_PRINT(" m | Velocity: "); DEBUG_PRINT2(currentState.estimatedVelocity, 2);
                DEBUG_PRINTLN(" m/s");
                logDebugMessage(">>> APOGEE DETECTED - DESCENDING <<<");
                #endif
            }
            #endif
        }
            break;
            
        case DESCENT: {
            // ================================================================
            // LANDING DETECTION: DESCENT → LANDED
            // altitude and velocity must be STABLE
            // Check if values barely change over last N samples
            // ================================================================
            
            bool lowAltitude = (currentState.estimatedAltitude < LANDING_ALT_THRESHOLD);
            
            #ifdef ENABLE_PHASE_BUFFER
            bool isStable = isStableFor(LANDING_SAMPLES_CHECK, 
                                       LANDING_ALT_CHANGE_MAX, 
                                       LANDING_VEL_CHANGE_MAX);
            
            if (lowAltitude && isStable) {
                currentPhase = LANDED;
                phaseStartTime = now;
                
                #ifdef DEBUG_PHASE_TRANSITIONS
                DEBUG_PRINTLN(">>> LANDED <<<");
                DEBUG_PRINT("Final Alt: "); DEBUG_PRINT2(currentState.estimatedAltitude, 2);
                DEBUG_PRINT(" m | Max Alt: "); DEBUG_PRINT2(maxAltitude, 2);
                DEBUG_PRINTLN(" m");
                logDebugMessage(">>> LANDED <<<");
                logDebugMessage("Final Altitude: ", currentState.estimatedAltitude, 2);
                logDebugMessage("Max Altitude: ", maxAltitude, 2);
                #endif
            }
            #else
            // Fallback: simple threshold check (less robust)
            bool lowVelocity = (abs(currentState.estimatedVelocity) < 3.0f);
            
            if (lowAltitude && lowVelocity) {
                if (lastMovementTime == 0) {
                    lastMovementTime = now;
                } else if ((now - lastMovementTime) > 2000000) {
                    currentPhase = LANDED;
                    phaseStartTime = now;
                    
                    #ifdef DEBUG_PHASE_TRANSITIONS
                    DEBUG_PRINTLN(">>> LANDED <<<");
                    DEBUG_PRINT("Final Alt: "); DEBUG_PRINT2(currentState.estimatedAltitude, 2);
                    DEBUG_PRINT(" m | Max Alt: "); DEBUG_PRINT2(maxAltitude, 2);
                    DEBUG_PRINTLN(" m");
                    logDebugMessage(">>> LANDED <<<");
                    logDebugMessage("Final Altitude: ", currentState.estimatedAltitude, 2);
                    logDebugMessage("Max Altitude: ", maxAltitude, 2);
                    #endif
                }
            } else {
                lastMovementTime = 0;
            }
            #endif
        }
            break;
            
        case LANDED:
            // Stay in landed state
            break;
    }
}

// ============================================================================
// CONTROL CHECK
// ============================================================================

bool shouldControl() {
    // Only control airbrakes during COASTING phase
    return (currentPhase == COASTING);
}
