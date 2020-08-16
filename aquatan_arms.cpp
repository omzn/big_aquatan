#include "aquatan_arms.h"

AquatanArms::AquatanArms(Servo *p, Servo *t) {
  left_servo = p;
  right_servo = t;
  _left_center = 90 + (LEFT_MAX_DEG + LEFT_MIN_DEG) / 2;
  _right_center = 90 + (RIGHT_MAX_DEG + RIGHT_MIN_DEG) / 2;
  _slowleft_moving = _slowright_moving = 0;
}

void AquatanArms::begin() {
  left_servo->setPeriodHertz(50);  // Standard 50hz servo
  right_servo->setPeriodHertz(50); // Standard 50hz servo
  left_servo->attach(PIN_SERVO2);
  right_servo->attach(PIN_SERVO4);
  left(0);
  right(0);
}

void AquatanArms::begin(int16_t leftdeg, int16_t rightdeg) {
  left_servo->setPeriodHertz(50);  // Standard 50hz servo
  right_servo->setPeriodHertz(50); // Standard 50hz servo
  left_servo->attach(PIN_SERVO2);
  right_servo->attach(PIN_SERVO4);
  left(leftdeg);
  right(rightdeg);
}

uint8_t AquatanArms::setSlowLeft(int16_t ratio) {
  if (!_slowleft_moving) {
    _slowleft_moving = 1;
    int16_t convdeg = map(ratio, -100, 100, LEFT_MIN_DEG, LEFT_MAX_DEG);
    _slowleft_target_deg = constrain(convdeg, LEFT_MIN_DEG, LEFT_MAX_DEG);
    return 1;
  } else {
    return 0;
  }
}

uint8_t AquatanArms::slowLeft() {
  if (_slowleft_moving) {
    if (_slowleft_target_deg < _left_deg) {
      _left_deg--;
    } else if (_slowleft_target_deg > _left_deg) {
      _left_deg++;
    } else {
      _slowleft_moving = 0;
      return 1;
    }
    //    left_servo->attach(PIN_SERVO2);
    left_servo->write(_left_center + (_left_deg * LEFT_DIRECTION));
    //    left_servo->detach();
  }
  return 0;
}

void AquatanArms::left(int16_t deg) {
  deg = constrain(deg, -90, 90);
  _left_deg = deg;
  //  left_servo->attach(PIN_SERVO2);
  left_servo->write(_left_center + (_left_deg * LEFT_DIRECTION));
  //  left_servo->detach();
}

int16_t AquatanArms::left() { return _left_deg; }

int16_t AquatanArms::leftRatio() {
  return map(_left_deg, LEFT_MIN_DEG, LEFT_MAX_DEG, -100, 100);
}

void AquatanArms::right(int16_t deg) {
  deg = constrain(deg, -90, 90);
  _right_deg = deg;
  //  right_servo->attach(PIN_SERVO4);
  right_servo->write(_right_center + (_right_deg * RIGHT_DIRECTION));
  //  right_servo->detach();
}

int16_t AquatanArms::right() { return _right_deg; }

int16_t AquatanArms::rightRatio() {
  return map(_right_deg, RIGHT_MIN_DEG, RIGHT_MAX_DEG, -100, 100);
}

uint8_t AquatanArms::setSlowRight(int16_t ratio) {
  if (!_slowright_moving) {
    _slowright_moving = 1;
    int16_t convdeg = map(ratio, -100, 100, RIGHT_MIN_DEG, RIGHT_MAX_DEG);
    _slowright_target_deg = constrain(convdeg, RIGHT_MIN_DEG, RIGHT_MAX_DEG);
    return 1;
  } else {
    return 0;
  }
}
// (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// (-100 - (-100)) * (70 - (-70)) = 0 / 130
uint8_t AquatanArms::slowRight() {
  if (_slowright_moving) {
    if (_slowright_target_deg < _right_deg) {
      _right_deg--;
    } else if (_slowright_target_deg > _right_deg) {
      _right_deg++;
    } else {
      _slowright_moving = 0;
      return 1;
    }
    //    right_servo->attach(PIN_SERVO4);
    right_servo->write(_right_center + (_right_deg * RIGHT_DIRECTION));
    //    right_servo->detach();
  }
  return 0;
}

/* ratio: -100 - 100, time: ms */
uint8_t AquatanArms::setMoveLeft(int16_t ratio, uint16_t time) {
  if (!_slowleft_moving) {
    _slowleft_moving = 1;
    int16_t convdeg = map(ratio, -100, 100, LEFT_MIN_DEG, LEFT_MAX_DEG);
    _slowleft_target_deg = constrain(convdeg, LEFT_MIN_DEG, LEFT_MAX_DEG);
    _left_interval = (time / (abs(_slowleft_target_deg - _left_deg) + 1)) + 1;
    return 1;
  } else {
    return 0;
  }
}

uint8_t AquatanArms::updateLeft() {
  static uint32_t prev_millis = 0;
  if (millis() - prev_millis > _left_interval) {
    prev_millis = millis();
    if (_slowleft_moving) {
      if (_slowleft_target_deg < _left_deg) {
        _left_deg--;
      } else if (_slowleft_target_deg > _left_deg) {
        _left_deg++;
      } else {
        _slowleft_moving = 0;
        return 1;
      }
      //    left_servo->attach(PIN_SERVO2);
      left_servo->write(_left_center + (_left_deg * LEFT_DIRECTION));
      //    left_servo->detach();
    }
  }
  return 0;
}

/* ratio: -100 - 100, time: ms */
uint8_t AquatanArms::setMoveRight(int16_t ratio, uint16_t time) {
  if (!_slowright_moving) {
    _slowright_moving = 1;
    int16_t convdeg = map(ratio, -100, 100, RIGHT_MIN_DEG, RIGHT_MAX_DEG);
    _slowright_target_deg = constrain(convdeg, RIGHT_MIN_DEG, RIGHT_MAX_DEG);
    _right_interval =
        (time / (abs(_slowright_target_deg - _right_deg) + 1)) + 1;
    return 1;
  } else {
    return 0;
  }
}

uint8_t AquatanArms::updateRight() {
  static uint32_t prev_millis = 0;
  if (millis() - prev_millis > _right_interval) {
    prev_millis = millis();
    if (_slowright_moving) {
      if (_slowright_target_deg < _right_deg) {
        _right_deg--;
      } else if (_slowright_target_deg > _right_deg) {
        _right_deg++;
      } else {
        _slowright_moving = 0;
        return 1;
      }
      //    right_servo->attach(PIN_SERVO4);
      right_servo->write(_right_center + (_right_deg * RIGHT_DIRECTION));
      //    right_servo->detach();
    }
  }
  return 0;
}

uint8_t AquatanArms::isMovingLeft() {
  return _slowleft_moving;
}

uint8_t AquatanArms::isMovingRight() {
  return _slowright_moving;  
}
