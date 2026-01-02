#ifndef CONFIG_H
#define CONFIG_H

// =====================
// IMU Configuration
// =====================
// Pin definitions
#define IMU_DRDY_PIN 20

// I2C addresses
#define IMU_ADDRESS 0x6B

#define IMU_INTERVAL 5000          // Microseconds between IMU reads (~200 Hz)

// =====================
// Barometer Configuration
// =====================
#define BARO_INTERVAL 10000        // Microseconds between Baro reads (~100 Hz)

// =====================
// GPS Configuration
// =====================
//#define GPS_BAUD_DEFAULT 9600
//#define GPS_BAUD_FAST    115200

// =====================
// Logging Configuration
// =====================
//#define LOG_BUFFER_SIZE 400

#endif
