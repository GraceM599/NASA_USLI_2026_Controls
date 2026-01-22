//
// Created by mihir on 1/20/2026.
//

#ifndef SWAMPLAUNCHTESTING_SERVODRIVER_H
#define SWAMPLAUNCHTESTING_SERVODRIVER_H

#include <Arduino.h>
#include <SCServo.h>
#include <iostream>
class ServoDriver {
    SMS_STS st;
public:
    int center;
    ServoDriver() {
        //initialize center to whatever the center pos should be and use it as refrence
        //for the rotateServo() call
    }
    void initializeServo() {
        Serial1.begin(1000000);
        //might be a different serial port
        // Serial1.begin(1000000, SERIAL_8N1, RX, TX); // custom serial port

        st.pSerial = &Serial1;
        if (!Serial1) {
            Serial.println("Port Issue with servo? ");
        }
    }

    void testServoConnection() {
        int ID = st.Ping(1);
        if (ID != -1) {
            Serial.print("Servo responded");
            delay(100);
        }
        else {
            Serial.println("Ping servo ID error!");
            delay(2000);
        }

    }
    //Get angle from a method in ApogeeController and input it here..?
    void rotateServo(int angle_deg, int speed, int acc) {
        int pos = angle_deg * (4095.0 / 300.0);
        st.WritePos(1, pos, speed, acc);
        //Also a synchronous write but i think theres just one servo
    }



};

#endif //SWAMPLAUNCHTESTING_SERVODRIVER_H