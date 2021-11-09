#ifndef AQUATAN_EYE_H
#define AQUATAN_EYE_H

#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)

#define EYE_HEIGHT   16
#define EYE_WIDTH    48

#define BLINK_FRAME  4
#define MOVE_FRAME  4

enum {EYE_IDLE = 0, EYE_BLINK, EYE_MOVE, EYE_WINK, EYE_SLEEP, EYE_LAST};
enum {BLINK_NONE = 0, BLINK_CLOSE, BLINK_OPEN};
enum {SHAPE_NORMAL = 0, SHAPE_ZITO, SHAPE_LT, SHAPE_GT, SHAPE_GURUGURU, SHAPE_HORI, SHAPE_LAST};

class AquatanEye {
private:
  Adafruit_SSD1306 *disp;

  int width_adj = 0, height_adj = 0;

  int _mode = EYE_IDLE; 
  int move_diff_x, target_dir_x, current_dir_x;
  int move_diff_y, target_dir_y, current_dir_y;
  int move_frame = 0;

  int _blink = BLINK_NONE;
  int blink_frame = 0;

  int _shape = SHAPE_NORMAL;
public:
  AquatanEye(Adafruit_SSD1306 *d);
  void begin();
  void begin(int,int);
  void shape(int);
  int shape();
  void show(int,int,int);
  void mode(int m);
  int mode();
  int move();
  int blink();
  int blink_st();
  void setMoveTarget(int,int);
};

class AquatanEyes {
private:
  AquatanEye *_left, *_right;
  int _shape = SHAPE_NORMAL;
  int _mode = EYE_IDLE; 
  int _blink = BLINK_NONE;
  uint32_t _blinktimer = 0;
  uint32_t _movetimer = 0;
  int fps = 24;
public:
  AquatanEyes(AquatanEye *l, AquatanEye *r);
  void begin();
  void begin(int,int,int,int);
  void shape(int);
  void shape(int, int);
  int shape();
  void show(int,int,int);
  void mode(int m);
  int mode();
  void move();
  void blink();
  void setMoveTarget(int,int);
  void randomBlinking(int move);
};

#endif