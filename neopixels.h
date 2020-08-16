#ifndef NEOPIXELS_H
#define NEOPIXELS_H

#include "Arduino.h"
#include <NeoPixelBus.h>

#define FPS 24

#define LEDCOLOR_RED    RgbColor(30,0,0)
#define LEDCOLOR_YELLOW RgbColor(20,30,5)
#define LEDCOLOR_BLUE   RgbColor(0,0,30)
#define LEDCOLOR_CYAN   RgbColor(0,20,20)
#define LEDCOLOR_GREEN  RgbColor(0,30,0)
#define LEDCOLOR_MAGENTA RgbColor(20,0,30)
#define LEDCOLOR_WHITE  RgbColor(15,25,25)

typedef enum {LED_OFF, LED_ON, LED_FADE, LED_BLINK} LED_mode_t;

// 通常 1  検知時 0 

class NeoPixels {
  public:
    NeoPixels(NeoPixelBus<NeoRgbFeature, Neo800KbpsMethod> *leds);
    void addPixel(uint8_t idx);
    void mode(LED_mode_t m);
    void period(float time);
    LED_mode_t mode();
    void color(RgbColor color);
    void update();

  protected:
    NeoPixelBus<NeoRgbFeature, Neo800KbpsMethod> *_leds;
    uint8_t _pixel_array[60];
    uint8_t _num_pixels = 0;
    LED_mode_t _mode = LED_OFF;
    RgbColor _color;
    uint16_t _frame = 0;
    uint16_t _frame_max;
};

#endif
