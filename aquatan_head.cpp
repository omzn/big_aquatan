#include "aquatan_head.h"
//#include "aquatan_esp32.h"

AquatanHead::AquatanHead(Servo *t) {
//  pan_servo = p;
  tilt_servo = t;
  _tilt_center = 90 + (TILT_MAX_DEG + TILT_MIN_DEG) / 2;
  _slowtilt_moving = 0;
}

void AquatanHead::begin() {
  tilt_servo->setPeriodHertz(50); // Standard 50hz servo
  tilt_servo->attach(PIN_SERVO1);
  tilt(0);
}

void AquatanHead::begin(int16_t tiltdeg) {
  tilt_servo->setPeriodHertz(50); // Standard 50hz servo
  tilt_servo->attach(PIN_SERVO1);
  tilt(tiltdeg);
}

void AquatanHead::setMinMax(int16_t tmin, int16_t tmax) {
  _tilt_min_deg = tmin;
  _tilt_max_deg = tmax;
  _tilt_center = 90 + (_tilt_max_deg + _tilt_min_deg) / 2;    
}

void AquatanHead::tilt(int16_t deg) {
  deg = constrain(deg, -90, 90);
  _tilt_deg = deg;
  tilt_servo->write(_tilt_center + (_tilt_deg * TILT_DIRECTION));
}

int16_t AquatanHead::tilt() { return _tilt_deg; }

int16_t AquatanHead::tiltRatio() { return map(_tilt_deg,_tilt_min_deg,_tilt_max_deg,-100,100); }

uint8_t AquatanHead::setSlowTilt(int16_t ratio) {
  if (!_slowtilt_moving) {
    _slowtilt_moving = 1;
    int16_t convdeg = map(ratio, -100, 100, _tilt_min_deg - (_tilt_min_deg + _tilt_max_deg) / 2, _tilt_max_deg - (_tilt_min_deg + _tilt_max_deg) / 2 );
    _slowtilt_target_deg = constrain(convdeg, _tilt_min_deg - (_tilt_min_deg + _tilt_max_deg) / 2, _tilt_max_deg - (_tilt_min_deg + _tilt_max_deg) / 2);
    return 1;
  } else {
    return 0;
  }
}
// (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// (-100 - (-100)) * (70 - (-70)) = 0 / 130  
uint8_t AquatanHead::slowTilt() {
  if (_slowtilt_moving) {
    if (_slowtilt_target_deg < _tilt_deg) {
      _tilt_deg--;
    } else if (_slowtilt_target_deg > _tilt_deg) {
      _tilt_deg++;
    } else {
      _slowtilt_moving = 0;
      return 1;
    }
    tilt_servo->write(_tilt_center + (_tilt_deg * TILT_DIRECTION));
  }
  return 0;
}

/* ratio: -100 - 100, time: ms */
uint8_t AquatanHead::setMoveTilt(int16_t ratio, uint16_t time) {
  if (!_slowtilt_moving) {
    _slowtilt_moving = 1;
    int16_t convdeg = map(ratio, -100, 100, _tilt_min_deg - (_tilt_min_deg + _tilt_max_deg) / 2 , _tilt_max_deg - (_tilt_min_deg + _tilt_max_deg) / 2 );
    _slowtilt_target_deg = constrain(convdeg, _tilt_min_deg - (_tilt_min_deg + _tilt_max_deg) / 2, _tilt_max_deg - (_tilt_min_deg + _tilt_max_deg) / 2);
    _tilt_interval =
        (time / (abs(_slowtilt_target_deg - _tilt_deg) + 1)) + 1;
    return 1;
  } else {
    return 0;
  }
}

uint8_t AquatanHead::updateTilt() {
  static uint32_t prev_millis = 0;
  if (millis() - prev_millis > _tilt_interval) {
    prev_millis = millis();
    if (_slowtilt_moving) {
      if (_slowtilt_target_deg < _tilt_deg) {
        _tilt_deg--;
      } else if (_slowtilt_target_deg > _tilt_deg) {
        _tilt_deg++;
      } else {
        _slowtilt_moving = 0;
        return 1;
      }
      //    tilt_servo->attach(PIN_SERVO4);
      tilt_servo->write(_tilt_center + (_tilt_deg * TILT_DIRECTION));
      //    tilt_servo->detach();
    }
  }
  return 0;
}

uint8_t AquatanHead::isMovingTilt() {
  return _slowtilt_moving;  
}
