#include "actionqueue.h"

actionQueue::actionQueue(AquatanEyes *e, AquatanArms *a, AquatanHead *h, NeoPixels *c) {
  _eyes = e;
  _arms = a;
  _head = h;
  _cheek = c;
}

void actionQueue::queueEyes(int l_shape,int r_shape, int m = EYE_IDLE) {
  qlast = (qlast + 1) % MAX_QUEUE_SIZE;
  _queue[qlast].type = ACT_EYES;
  _queue[qlast].eyes_leftshape = l_shape;
  _queue[qlast].eyes_rightshape = r_shape;
  _queue[qlast].eyes_mode = m;
}

void actionQueue::queueArms(int left, int right, int time) {
  qlast = (qlast + 1) % MAX_QUEUE_SIZE;
  _queue[qlast].type = ACT_ARMS;
  _queue[qlast].arm_left = left;
  _queue[qlast].arm_right = right;
  _queue[qlast].arm_time = time;
}

void actionQueue::queueArmLeft(int left, int time) {
  qlast = (qlast + 1) % MAX_QUEUE_SIZE;
  _queue[qlast].type = ACT_ARM_LEFT;
  _queue[qlast].arm_left = left;
  _queue[qlast].arm_time = time;
}

void actionQueue::queueArmRight(int right, int time) {
  qlast = (qlast + 1) % MAX_QUEUE_SIZE;
  _queue[qlast].type = ACT_ARM_RIGHT;
  _queue[qlast].arm_right = right;
  _queue[qlast].arm_time = time;
}

void actionQueue::queueHeadTilt(int tilt) {
  qlast = (qlast + 1) % MAX_QUEUE_SIZE;
  _queue[qlast].type = ACT_HEAD_TILT;
  _queue[qlast].head_tilt = tilt;
}

void actionQueue::queueCheek(LED_mode_t m, RgbColor color) {
  qlast = (qlast + 1) % MAX_QUEUE_SIZE;
  _queue[qlast].type = ACT_CHEEK;
  _queue[qlast].cheek_mode = m;
  _queue[qlast].cheek_color = color;
}

void actionQueue::queueWait(int t) {
  qlast = (qlast + 1) % MAX_QUEUE_SIZE;
  _queue[qlast].type = ACT_WAIT;
  _queue[qlast].wait = t;
}

uint8_t actionQueue::isEmpty() {
  return qtop == qlast ? 1 : 0;
}

uint8_t actionQueue::dequeue() {
  static uint32_t prev_millis = 0;
  static uint32_t time_wait = 0;
  if (qtop == qlast) {
    return 0;
  }
  if (_arms->isMovingLeft() || _arms->isMovingRight() || 
      _head->isMovingTilt()) {
    return 0;
  }
  if (prev_millis > 0 && millis() - prev_millis < time_wait) {
    return 0;
  }
  prev_millis = 0;
  qtop = (qtop + 1) % MAX_QUEUE_SIZE;
  switch (_queue[qtop].type) {
  case ACT_WAIT:
    prev_millis = millis();
    time_wait = _queue[qtop].wait;
    break;
  case ACT_EYES:
    _eyes->shape(_queue[qtop].eyes_leftshape,_queue[qtop].eyes_rightshape);
    _eyes->mode(_queue[qtop].eyes_mode);
    break;
  case ACT_ARMS:
    _arms->setMoveLeft(_queue[qtop].arm_left, _queue[qtop].arm_time);
    _arms->setMoveRight(_queue[qtop].arm_right, _queue[qtop].arm_time);
    break;
  case ACT_ARM_LEFT:
    _arms->setMoveLeft(_queue[qtop].arm_left, _queue[qtop].arm_time);
    break;
  case ACT_ARM_RIGHT:
    _arms->setMoveRight(_queue[qtop].arm_right, _queue[qtop].arm_time);
    break;
  case ACT_HEAD_TILT:
    _head->setMoveTilt(_queue[qtop].head_tilt);
    break;
  case ACT_CHEEK:
    _cheek->mode(_queue[qtop].cheek_mode);
    _cheek->color(_queue[qtop].cheek_color);
    break;
  default:
    break;
  }
  return 1;
}

void actionQueue::clear() { qtop = qlast = -1; }
