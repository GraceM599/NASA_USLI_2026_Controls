/*
 * ============================================================================
 * GPS - Implementation File
 * ============================================================================
 * Handles GPS initialization, NMEA parsing, and data access
 */

#include "GPS.h"
#include "State.h"
#include "Debug.h"

GPS::GPS(HardwareSerial &serialPort)
    : GPSSerial(serialPort), latitude(0.0), longitude(0.0), satellites(0), fix(false), altitude(0.0), speed(0.0) {}

void GPS::begin() {
    setupGPS();
}

void GPS::setupGPS() {
    DEBUG_PRINTLN("Initializing GPS...");
    
    // Initialize GPS serial port at 9600 (default)
    GPSSerial.begin(9600);
    //while (!GPSSerial) delay(10);  // Wait for serial to be ready
    
    // Change to 115200 baud
    GPSSerial.println(PMTK_SET_BAUD_115200);  // Use println instead of print!
    delay(100);
    GPSSerial.end();
    delay(100);
    
    // Reopen at 115200
    GPSSerial.begin(115200);
    delay(100);
    
    DEBUG_PRINTLN("Now listening at 115200...");
    
    // Set output mode to GGA only
    GPSSerial.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);  // Use println!
    delay(100);
    
    // Set update rate to 10 Hz
    GPSSerial.println(PMTK_SET_NMEA_UPDATE_10HZ);  // Use println!
    delay(100);
    
    DEBUG_PRINTLN("GPS Initialization done!");
}

void GPS::update() {
    static String sentenceBuffer = "";
    
    // Read all available GPS data
    while (GPSSerial.available()) {
        char c = GPSSerial.read();
        
        if (c == '\n') {
            // Parse the complete sentence
            if (sentenceBuffer.startsWith("$GPGGA") || sentenceBuffer.startsWith("$GNGGA")) {
                parseGGA(sentenceBuffer);
            } else if (sentenceBuffer.startsWith("$GPRMC") || sentenceBuffer.startsWith("$GNRMC")) {
                parseRMC(sentenceBuffer);
            }
            sentenceBuffer = "";
        } else if (c != '\r') {
            sentenceBuffer += c;
        }
        
        // Prevent buffer overflow
        if (sentenceBuffer.length() > 120) {
            sentenceBuffer = "";
        }
    }
}

void GPS::parseGGA(const String &sentence) {
    // Simple comma-based parsing
    // $GPGGA,time,lat,N/S,lon,E/W,fix,sats,hdop,alt,M,...
    // Fields: 0     1    2   3    4   5   6   7    8    9   10
    
    int commaPositions[15];
    int commaCount = 0;
    
    // Find all comma positions
    for (unsigned int i = 0; i < sentence.length() && commaCount < 15; i++) {
        if (sentence.charAt(i) == ',') {
            commaPositions[commaCount++] = i;
        }
    }
    
    // Need at least 10 commas to have all fields
    if (commaCount < 10) {
        Serial.println("Not enough commas!");
        return;
    }
    
    // Extract fields by position
    // Field 2: Latitude (between comma 1 and 2)
    String latStr = sentence.substring(commaPositions[1] + 1, commaPositions[2]);
    // Field 3: N/S (between comma 2 and 3)
    String latDir = sentence.substring(commaPositions[2] + 1, commaPositions[3]);
    // Field 4: Longitude (between comma 3 and 4)
    String lonStr = sentence.substring(commaPositions[3] + 1, commaPositions[4]);
    // Field 5: E/W (between comma 4 and 5)
    String lonDir = sentence.substring(commaPositions[4] + 1, commaPositions[5]);
    // Field 6: Fix (between comma 5 and 6)
    String fixStr = sentence.substring(commaPositions[5] + 1, commaPositions[6]);
    // Field 7: Satellites (between comma 6 and 7)
    String satStr = sentence.substring(commaPositions[6] + 1, commaPositions[7]);
    // Field 9: Altitude (between comma 8 and 9)
    String altStr = sentence.substring(commaPositions[8] + 1, commaPositions[9]);
    
    // Parse fix
    if (fixStr.length() > 0) {
        fix = (fixStr.toInt() > 0);
    }
    
    // Parse satellites
    if (satStr.length() > 0) {
        satellites = satStr.toInt();
    }
    
    // Parse latitude (DDMM.MMMM format)
    if (latStr.length() > 0) {
        float lat = latStr.toFloat();
        int degrees = (int)(lat / 100);
        float minutes = lat - (degrees * 100);
        latitude = degrees + (minutes / 60.0);
        if (latDir == "S") latitude = -latitude;
    }
    
    // Parse longitude (DDDMM.MMMM format)
    if (lonStr.length() > 0) {
        float lon = lonStr.toFloat();
        int degrees = (int)(lon / 100);
        float minutes = lon - (degrees * 100);
        longitude = degrees + (minutes / 60.0);
        if (lonDir == "W") longitude = -longitude;
    }
    
    // Parse altitude
    if (altStr.length() > 0) {
        altitude = altStr.toFloat();
    }
}

void GPS::parseRMC(const String &sentence) {
    int fieldCount = 0;
    int lastIndex = 0;
    String fields[13];

    // Split by commas
    for (unsigned int i = 0; i < sentence.length(); i++) {
        if (sentence.charAt(i) == ',' || sentence.charAt(i) == '*') {
            fields[fieldCount++] = sentence.substring(lastIndex, i);
            lastIndex = i + 1;
            if (fieldCount >= 13) break;
        }
    }

    // Field 7: Speed in knots
    if (fields[7].length() > 0) {
        float speedKnots = fields[7].toFloat();
        speed = speedKnots * 0.514444;  // Convert to m/s
    } else {
        speed = 0.0;
    }
}

// Getter methods
float GPS::getLatitude() const { return latitude; }
float GPS::getLongitude() const { return longitude; }
uint8_t GPS::getSatellites() const { return satellites; }
bool GPS::hasFix() const { return fix; }
float GPS::getAltitude() const { return altitude; }
float GPS::getSpeed() const { return speed; }

// Update State struct with GPS data
void GPS::updateState(State& currentState) {
    currentState.gpsLat = latitude;
    currentState.gpsLon = longitude;
    currentState.gpsAltitude = altitude;
    currentState.gpsSpeed = speed;
    currentState.gpsSatellites = satellites;
    currentState.gpsHasFix = fix;
}
