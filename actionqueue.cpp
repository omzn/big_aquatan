#include "actionqueue.h"

actionQueue::actionQueue(AquatanEyes *e, AquatanArms *a, Camera *h) {
  _eyes = e;
  _arms = a;
  _head = h;
}

void actionQueue::queueEyes(int shape) {
  qlast = (qlast + 1) % MAX_QUEUE_SIZE;
  _queue[qlast].type = ACT_EYES;
  _queue[qlast].eyes_shape = shape;
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

void actionQueue::queueHead(int pan, int tilt) {
  qlast = (qlast + 1) % MAX_QUEUE_SIZE;
  _queue[qlast].type = ACT_HEAD;
  _queue[qlast].head_pan = pan;
  _queue[qlast].head_tilt = tilt;
}

void actionQueue::queueHeadPan(int pan) {
  qlast = (qlast + 1) % MAX_QUEUE_SIZE;
  _queue[qlast].type = ACT_HEAD_PAN;
  _queue[qlast].head_pan = pan;
}
void actionQueue::queueHeadTilt(int tilt) {
  qlast = (qlast + 1) % MAX_QUEUE_SIZE;
  _queue[qlast].type = ACT_HEAD_TILT;
  _queue[qlast].head_tilt = tilt;
}

uint8_t actionQueue::dequeue() {
  if (qtop == qlast) {
    return 0;
  }
  if (_arms->isMovingLeft() || _arms->isMovingRight() || _head->isMovingPan() ||
      _head->isMovingTilt()) {
    return 0;
  }
  qtop = (qtop + 1) % MAX_QUEUE_SIZE;
  switch (_queue[qtop].type) {
  case ACT_EYES:
    _eyes->shape(_queue[qtop].eyes_shape);
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
  case ACT_HEAD:
    _head->setMovePan(_queue[qtop].head_pan);
    _head->setMoveTilt(_queue[qtop].head_tilt);
    break;
  case ACT_HEAD_PAN:
    _head->setMovePan(_queue[qtop].head_pan);
    break;
  case ACT_HEAD_TILT:
    _head->setMoveTilt(_queue[qtop].head_tilt);
    break;
  default:
    break;
  }
  return 1;
}

void actionQueue::clear() { qtop = qlast = -1; }
