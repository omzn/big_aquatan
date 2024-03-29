#ifndef ACTIONQUEUE_H
#define ACTIONQUEUE_H

#define MAX_QUEUE_SIZE (70)

#include "aquatan_arms.h"
#include "aquatan_eye.h"
#include "camera.h"
#include "neopixels.h"

enum {
  ACT_EYES = 0,
  ACT_HEAD,
  ACT_HEAD_PAN,
  ACT_HEAD_TILT,
  ACT_ARMS,
  ACT_ARM_LEFT,
  ACT_ARM_RIGHT,
  ACT_CHEEK,
  ACT_WAIT
};

typedef struct node {
  uint8_t type;
  int eyes_leftshape, eyes_rightshape, eyes_mode;
  int arm_left, arm_right, arm_time;
  int head_pan, head_tilt;
  LED_mode_t cheek_mode;
  RgbColor cheek_color;
  int wait;
} Node;

class actionQueue {
public:
  actionQueue(AquatanEyes *e, AquatanArms *a, Camera *h,
              NeoPixels *c); //, Charger *c);
  void queueEyes(int l_shape, int r_shape, int m);
  void queueArms(int left, int right, int time);
  void queueArmLeft(int left, int time);
  void queueArmRight(int right, int time);
  void queueHead(int pan, int tilt);
  void queueHeadPan(int pan);
  void queueHeadTilt(int tilt);
  void queueCheek(LED_mode_t m, RgbColor color);
  void queueWait(int w);
  uint8_t dequeue();
  void clear();
  uint8_t isEmpty();

protected:
  Node _queue[MAX_QUEUE_SIZE];
  int8_t qlast = -1;
  int8_t qtop = -1;
  AquatanEyes *_eyes;
  AquatanArms *_arms;
  Camera *_head;
  NeoPixels *_cheek;
  //  Charger *_charger;
};

#endif