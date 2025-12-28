/*
 * ============================================================================
 * GPS - Header File
 * ============================================================================
 * Handles GPS initialization, NMEA parsing, and data access
 */

#ifndef GPS_H
#define GPS_H

#include <Arduino.h>

// Forward declaration
struct State;

// ============================================================================
// GPS CONFIGURATION
// ============================================================================

// GPS Commands
#define PMTK_SET_NMEA_OUTPUT_OFF      "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_OUTPUT_RMCONLY  "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA   "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_OUTPUT_ALLDATA  "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_OUTPUT_GGAVTG   "$PMTK314,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_OUTPUT_VTGONLY  "$PMTK314,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_GGAONLY  "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"

#define PMTK_SET_NMEA_UPDATE_1HZ   "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_2HZ   "$PMTK220,500*2B"
#define PMTK_SET_NMEA_UPDATE_5HZ   "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ  "$PMTK220,100*2F"

#define PMTK_SET_BAUD_9600    "$PMTK251,9600*17"
#define PMTK_SET_BAUD_115200  "$PMTK251,115200*1F"

// ============================================================================
// GPS CLASS
// ============================================================================

class GPS {
public:
    GPS(HardwareSerial &serialPort);
    void begin();
    void update();
    void updateState(State& currentState);  // Update State struct with GPS data

    // Getter methods
    float getLatitude() const;
    float getLongitude() const;
    uint8_t getSatellites() const;
    bool hasFix() const;
    float getAltitude() const;
    float getSpeed() const;

private:
    HardwareSerial &GPSSerial;

    // Variables to store parsed data
    float latitude;
    float longitude;
    uint8_t satellites;
    bool fix;
    float altitude;
    float speed;

    void setupGPS();
    void parseGGA(const String &sentence);
    void parseRMC(const String &sentence);
};

#endif // GPS_H
