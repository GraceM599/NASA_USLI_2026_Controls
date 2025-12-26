#include <Arduino.h>

// ============================================================================
// DEBUG & CONFIGURATION SWITCHES
// ============================================================================

// Debug flags (comment out to disable)
#define DEBUG_SERIAL              // Print debug messages
#define DEBUG_PHASE_TRANSITIONS   // Print when phases change
#define DEBUG_SENSOR_READINGS     // Print sensor data periodically
// #define DEBUG_VERBOSE          // Very detailed logging (slow!)
// #define DEBUG_NO_SD            // Disable SD card (for bench testing)
// #define DEBUG_SIMULATE_FLIGHT  // Inject fake sensor data for testing

// Feature flags
#define ENABLE_IMU_TARE          // Perform IMU bias removal on startup
#define ENABLE_BARO_TARE         // Record ground altitude
#define ENABLE_PHASE_BUFFER      // Use buffer for phase detection
// #define ENABLE_GPS             // Enable GPS (not yet implemented)
// #define ENABLE_KALMAN          // Enable Kalman filters (not yet implemented)
// #define ENABLE_PID_CONTROL     // Enable PID controller (not yet implemented)

// Debug helper macros
#ifdef DEBUG_SERIAL
  #define DEBUG_PRINT(x)       Serial.print(x)
  #define DEBUG_PRINTLN(x)     Serial.println(x)
  #define DEBUG_PRINT2(x, y)   Serial.print(x, y)
  #define DEBUG_PRINTLN2(x, y) Serial.println(x, y)
  #define DEBUG_PRINTF(...)    Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINT2(x, y)
  #define DEBUG_PRINTLN2(x, y)
  #define DEBUG_PRINTF(...)
#endif

#ifdef DEBUG_VERBOSE
  #define VERBOSE_PRINT(x)    Serial.print(x)
  #define VERBOSE_PRINTLN(x)  Serial.println(x)
#else
  #define VERBOSE_PRINT(x)
  #define VERBOSE_PRINTLN(x)
#endif
