#include "camera.h"
//#include "aquatan_esp32.h"

Camera::Camera(Servo *p, Servo *t) {
  pan_servo = p;
  tilt_servo = t;
  _pan_center = 90 + (PAN_MAX_DEG + PAN_MIN_DEG) / 2;
  _tilt_center = 90 + (TILT_MAX_DEG + TILT_MIN_DEG) / 2;
  _slowpan_moving = _slowtilt_moving = 0;
}

void Camera::begin() {
  pan_servo->setPeriodHertz(50); // Standard 50hz servo
  pan_servo->attach(PIN_SERVO1);
  tilt_servo->setPeriodHertz(50); // Standard 50hz servo
  tilt_servo->attach(PIN_SERVO3);
  pan(0);
  tilt(0);
}

void Camera::begin(int16_t pandeg, int16_t tiltdeg) {
  pan_servo->setPeriodHertz(50); // Standard 50hz servo
  pan_servo->attach(PIN_SERVO1);
  tilt_servo->setPeriodHertz(50); // Standard 50hz servo
  tilt_servo->attach(PIN_SERVO3);
  pan(pandeg);
  tilt(tiltdeg);
}

void Camera::setMinMax(int16_t pmin, int16_t pmax, int16_t tmin, int16_t tmax) {
  _pan_min_deg = pmin;
  _pan_max_deg = pmax;
  _tilt_min_deg = tmin;
  _tilt_max_deg = tmax;
  _pan_center = 90 + (_pan_max_deg + _pan_min_deg) / 2;
  _tilt_center = 90 + (_tilt_max_deg + _tilt_min_deg) / 2;    
}

uint8_t Camera::setSlowPan(int16_t ratio) {
  if (!_slowpan_moving) {
    _slowpan_moving = 1;
    int16_t convdeg = map(ratio, -100, 100, PAN_MIN_DEG, PAN_MAX_DEG);
    _slowpan_target_deg = constrain(convdeg, PAN_MIN_DEG, PAN_MAX_DEG);
    return 1;
  } else {
    return 0;
  }
}

uint8_t Camera::slowPan() {
  if (_slowpan_moving) {
    if (_slowpan_target_deg < _pan_deg) {
      _pan_deg--;
    } else if (_slowpan_target_deg > _pan_deg) {
      _pan_deg++;
    } else {
      _slowpan_moving = 0;
      return 1;
    }
    pan_servo->write(_pan_center + (_pan_deg * PAN_DIRECTION));
  }
  return 0;
}

void Camera::pan(int16_t deg) {
  deg = constrain(deg, -90, 90);
  _pan_deg = deg;
  pan_servo->write(_pan_center + (_pan_deg * PAN_DIRECTION));
}

int16_t Camera::pan() { return _pan_deg; }

int16_t Camera::panRatio() { return map(_pan_deg,PAN_MIN_DEG,PAN_MAX_DEG,-100,100); }

void Camera::tilt(int16_t deg) {
  deg = constrain(deg, -90, 90);
  _tilt_deg = deg;
  tilt_servo->write(_tilt_center + (_tilt_deg * TILT_DIRECTION));
}

int16_t Camera::tilt() { return _tilt_deg; }

int16_t Camera::tiltRatio() { return map(_tilt_deg,TILT_MIN_DEG,TILT_MAX_DEG,-100,100); }

uint8_t Camera::setSlowTilt(int16_t ratio) {
  if (!_slowtilt_moving) {
    _slowtilt_moving = 1;
    int16_t convdeg = map(ratio, -100, 100, TILT_MIN_DEG, TILT_MAX_DEG);
    _slowtilt_target_deg = constrain(convdeg, TILT_MIN_DEG, TILT_MAX_DEG);
    return 1;
  } else {
    return 0;
  }
}
// (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// (-100 - (-100)) * (70 - (-70)) = 0 / 130  
uint8_t Camera::slowTilt() {
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
uint8_t Camera::setMovePan(int16_t ratio, uint16_t time) {
  if (!_slowpan_moving) {
    _slowpan_moving = 1;
    int16_t convdeg = map(ratio, -100, 100, PAN_MIN_DEG, PAN_MAX_DEG);
    _slowpan_target_deg = constrain(convdeg, PAN_MIN_DEG, PAN_MAX_DEG);
    _pan_interval = (time / (abs(_slowpan_target_deg - _pan_deg) + 1)) + 1;
    return 1;
  } else {
    return 0;
  }
}

uint8_t Camera::updatePan() {
  static uint32_t prev_millis = 0;
  if (millis() - prev_millis > _pan_interval) {
    prev_millis = millis();
    if (_slowpan_moving) {
      if (_slowpan_target_deg < _pan_deg) {
        _pan_deg--;
      } else if (_slowpan_target_deg > _pan_deg) {
        _pan_deg++;
      } else {
        _slowpan_moving = 0;
        return 1;
      }
      //    pan_servo->attach(PIN_SERVO2);
      pan_servo->write(_pan_center + (_pan_deg * PAN_DIRECTION));
      //    pan_servo->detach();
    }
  }
  return 0;
}

/* ratio: -100 - 100, time: ms */
uint8_t Camera::setMoveTilt(int16_t ratio, uint16_t time) {
  if (!_slowtilt_moving) {
    _slowtilt_moving = 1;
    int16_t convdeg = map(ratio, -100, 100, TILT_MIN_DEG, TILT_MAX_DEG);
    _slowtilt_target_deg = constrain(convdeg, TILT_MIN_DEG, TILT_MAX_DEG);
    _tilt_interval =
        (time / (abs(_slowtilt_target_deg - _tilt_deg) + 1)) + 1;
    return 1;
  } else {
    return 0;
  }
}

uint8_t Camera::updateTilt() {
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


uint8_t Camera::isMovingPan() {
  return _slowpan_moving;
}

uint8_t Camera::isMovingTilt() {
  return _slowpan_moving;  
}
