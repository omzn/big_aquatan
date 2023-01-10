#ifndef AQUATANHEAD_H
#define AQUATANHEAD_H

#include "Arduino.h"
#include <ESP32Servo.h>

#define PIN_SERVO1      (18)

#define SERVO_MIN_US   (500)
#define SERVO_MAX_US  (2400)
#define SERVO_SPEED     (50)

//#define PAN_DIRECTION  (-1)
#define TILT_DIRECTION (-1)

//#define PAN_MAX_DEG  ( 12)
//#define PAN_MIN_DEG  (-12)

#define TILT_MAX_DEG  (90-23)
#define TILT_MIN_DEG  (90-38)

class AquatanHead {
    public:
        AquatanHead(Servo *tilt_s);
        void begin();
        void begin(int16_t);
 //       void pan(int16_t);
 //       int16_t pan();
 //       int16_t panRatio();
 //       uint8_t setSlowPan(int16_t);
 //       uint8_t slowPan();
        void tilt(int16_t);
        void tiltRatio(int16_t);
        int16_t tilt();
        int16_t tiltRatio();
        uint8_t setSlowTilt(int16_t);
        uint8_t slowTilt();

 //       uint8_t setMovePan(int16_t ratio, uint16_t time = 120);
 //       uint8_t updatePan();
        uint8_t setMoveTilt(int16_t ratio, uint16_t time = 120);
        uint8_t updateTilt();
 //       uint8_t isMovingPan();
        uint8_t isMovingTilt();

        void setMinMax(int16_t tmin, int16_t tmax);

    private:
        Servo *tilt_servo;
        int16_t _tilt_center;
        int16_t _tilt_deg;
        int16_t _tilt_min_deg, _tilt_max_deg;
        int16_t _slowtilt_target_deg;
        uint8_t _slowtilt_moving;
        uint16_t _tilt_interval;
};

#endif