#ifndef AQUATAN_ARMS_H
#define AQUATAN_ARMS_H

#include "Arduino.h"
#include <ESP32Servo.h>

#define PIN_SERVO1      (18)
#define PIN_SERVO2      (19)
#define PIN_SERVO3      (23)
#define PIN_SERVO4      (26)

#define LEFT_DIRECTION  (-1)
#define RIGHT_DIRECTION  (1)

#define LEFT_MAX_DEG  (30)
#define LEFT_MIN_DEG  (-30)

#define RIGHT_MAX_DEG  (30)
#define RIGHT_MIN_DEG  (-30)

class AquatanArms {
    public:
        AquatanArms(Servo *left_s, Servo *right_s);
        void begin();
        void begin(int16_t, int16_t);
        void left(int16_t);
        int16_t left();
        int16_t leftRatio();
        uint8_t setSlowLeft(int16_t);
        uint8_t slowLeft();
        void right(int16_t);
        int16_t right();
        int16_t rightRatio();
        uint8_t setSlowRight(int16_t);
        uint8_t slowRight();

        uint8_t setMoveLeft(int16_t ratio, uint16_t time = 60);
        uint8_t updateLeft();
        uint8_t setMoveRight(int16_t ratio, uint16_t time = 60);
        uint8_t updateRight();
        uint8_t isMovingLeft();
        uint8_t isMovingRight();

        void setMinMax(int16_t,int16_t,int16_t,int16_t);

    private:
        Servo *left_servo, *right_servo;
        int16_t _left_center, _right_center;
        int16_t _left_max_deg = LEFT_MAX_DEG, _right_max_deg = RIGHT_MAX_DEG;
        int16_t _left_min_deg = LEFT_MIN_DEG, _right_min_deg = RIGHT_MIN_DEG;
        
        int16_t _left_deg, _right_deg;
        int16_t _slowleft_target_deg, _slowright_target_deg;
        uint8_t _slowleft_moving, _slowright_moving;
        uint16_t _left_interval, _right_interval;
};

#endif