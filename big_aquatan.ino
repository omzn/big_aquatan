/**************************************************************************
 This is an example for our Monochrome OLEDs based on SSD1306 drivers

 Pick one up today in the adafruit shop!
 ------> http://www.adafruit.com/category/63_98

 This example is for a 128x32 pixel display using I2C to communicate
 3 pins are required to interface (two I2C and one reset).

 Adafruit invests time and resources providing this open
 source code, please support Adafruit and open-source
 hardware by purchasing products from Adafruit!

 Written by Limor Fried/Ladyada for Adafruit Industries,
 with contributions from the open source community.
 BSD license, check license.txt for more information
 All text above, and the splash screen below must be
 included in any redistribution.
 **************************************************************************/

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <NeoPixelBus.h>
#include <Preferences.h>
#include <RTClib.h>
#include <SPI.h>
//#include <SparkFun_APDS9960.h>
//#include <WebServer.h>
//#include <WiFiManager.h>
#include <Wire.h>

#include "actionqueue.h"
#include "aquatan_arms.h"
#include "aquatan_eye.h"
#include "aquatan_head.h"
#include "big_aquatan.h"
#include "neopixels.h"
#include "ntp.h"

#define PIN_SDA1 (16)
#define PIN_SCL1 (17)
#define PIN_NEOPIXEL (5)
#define PIN_SERVO1 (18)
#define PIN_SERVO2 (26)
#define PIN_SERVO3 (19)
//#define PIN_SERVO4 (23)

#define PIN_SERIAL2_RX (32)
#define PIN_SERIAL2_TX (33)

#define PIN_MIC (36)

#define NEOPIXEL_LED_NUM (2)

Adafruit_SSD1306 oled1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 oled2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);

// Serial2.begin(115200, SERIAL_8N1, 32, 33);// Grove

AquatanEye eye_left(&oled1);
AquatanEye eye_right(&oled2);
AquatanEyes eyes(&eye_left, &eye_right);

Servo tilt, arm_left, arm_right;
AquatanHead head(&tilt);
AquatanArms arms(&arm_left, &arm_right);

NeoPixelBus<NeoRgbFeature, Neo800KbpsMethod> leds(NEOPIXEL_LED_NUM,
                                                  PIN_NEOPIXEL);
NeoPixels cheek(&leds);

//actionQueue actions(&eyes, &arms, &head, &cheek);

// WebServer webServer(80);
// NTP ntp("ntp.nict.jp");
// RTC_Millis rtc;

Preferences prefs;

String myhost;
int operation_mode;

uint32_t prev_time;
uint32_t prev_frame;

uint32_t voice_time;
uint32_t operation_timer;
uint32_t move_timer;

int16_t run_left = 100, run_right = -100;

enum {
  OPERATION_MANUAL = 0,
  OPERATION_FORWARD,
  OPERATION_BACKWARD,
  OPERATION_LEFT,
  OPERATION_RIGHT,
  OPERATION_SPIN,
  OPERATION_SLEEP
};

// Find the Peak-to-Peak Amplitude Function
int getSoundLevel() {
  static int preAmp = 0;
  // Time variables to find the peak-to-peak amplitude
  unsigned long startTime = millis(); // Start of sample window
  unsigned int PTPAmp = 0;
  int sampleTime = 20;

  // Signal variables to find the peak-to-peak amplitude
  unsigned int maxAmp = 0;
  unsigned int minAmp = 4095;

  // Find the max and min of the mic output within the 50 ms timeframe
  while (millis() - startTime < sampleTime) {
    int micOut = analogRead(PIN_MIC);
    if (micOut < 4096) // prevent erroneous readings
    {
      if (micOut > maxAmp) {
        maxAmp = micOut; // save only the max reading
      } else if (micOut < minAmp) {
        minAmp = micOut; // save only the min reading
      }
    }
  }

  PTPAmp = maxAmp - minAmp; // (max amp) - (min amp) = peak-to-peak amplitude
  int fill = map(PTPAmp, 50, 500, 0, 10);
  if (fill != preAmp) {
    preAmp = fill;
  }
//  Serial.print("vol:");
//  Serial.println(fill);
  return fill >= 10 ? 1 : 0;
}


void setup() {
  // WiFiManager wifiManager;

  Serial.begin(9600);
  Serial2.begin(115200, SERIAL_8N1, PIN_SERIAL2_RX, PIN_SERIAL2_TX);
  // rtc.begin(DateTime(2019, 12, 15, 15, 45, 0));
  prefs.begin("aquatan", false);

  myhost = prefs.getString("hostname", "bigaquatan");
  operation_mode =
      OPERATION_MANUAL;  // prefs.getUInt("operation_mode", OPERATION_MANUAL);

  leds.Begin();

  cheek.addPixel(0);
  cheek.addPixel(1);
  cheek.mode(LED_OFF);
  cheek.update();

  head.setMinMax(TILT_MIN_DEG, TILT_MAX_DEG);
  head.begin(0);
  arms.begin(0, 0);
  // arms.setMinMax(prefs.getShort("arm_left_min_deg", LEFT_MIN_DEG),
  //                prefs.getShort("arm_left_max_deg", LEFT_MAX_DEG),
  //                prefs.getShort("arm_right_min_deg", RIGHT_MIN_DEG),
  //                prefs.getShort("arm_right_max_deg", RIGHT_MAX_DEG));
  arms.setMinMax(LEFT_MIN_DEG, LEFT_MAX_DEG, RIGHT_MIN_DEG, RIGHT_MAX_DEG);

  Wire1.begin(PIN_SDA1, PIN_SCL1);
  eyes.begin(0, 0, 4, 0);

  cheek.color(LEDCOLOR_BLUE);
  cheek.mode(LED_FADE);
  cheek.period(2);

  eyes.shape(SHAPE_NORMAL);
  eyes.mode(EYE_IDLE);
  arms.leftRatio(50);
  arms.rightRatio(50);
  head.tiltRatio(0);
//  actions.queueHeadTilt(0);
//  actions.queueArmLeft(50, 50);
//  actions.queueArmRight(50, 50);
  move_timer = millis();
  operation_timer = millis();
}

void loop() {
  char op, val;

  if (Serial2.available()) {
    op = Serial2.read();
//    Serial.println(op);
    if (op) {
      if (millis() - operation_timer > 250) {
        operation_timer = millis();
        if (op == 'f' && operation_mode != OPERATION_FORWARD) {  // run forward
          Serial.println(op);
          operation_mode = OPERATION_FORWARD;
          cheek.color(LEDCOLOR_YELLOW);
          cheek.mode(LED_FADE);
          cheek.period(1);
          eyes.shape(SHAPE_GT, SHAPE_LT);
          eyes.mode(EYE_MOVE);
          eyes.setMoveTarget(0, 0);
          arms.leftRatio(run_left);
          arms.rightRatio(run_right);
          head.tiltRatio(0);
        } else if (op == 'b' &&
                   operation_mode != OPERATION_BACKWARD) {  // run backword
          Serial.println(op);
          operation_mode = OPERATION_BACKWARD;
          cheek.color(LEDCOLOR_RED);
          cheek.mode(LED_FADE);
          cheek.period(1);
          eyes.shape(SHAPE_ZITO);
          eyes.mode(EYE_MOVE);
          eyes.setMoveTarget(-1, 0);
          arms.leftRatio(-100);
          arms.rightRatio(-100);
          head.tiltRatio(-100);
        } else if (op == 'l' && operation_mode != OPERATION_LEFT) {  // run left
          Serial.println(op);
          operation_mode = OPERATION_LEFT;
          cheek.color(LEDCOLOR_GREEN);
          cheek.mode(LED_FADE);
          cheek.period(1);
          eyes.shape(SHAPE_ZITO);
          eyes.mode(EYE_MOVE);
          eyes.setMoveTarget(0, -1);
          arms.leftRatio(100);
          arms.rightRatio(0);
          head.tiltRatio(0);
        } else if (op == 'r' &&
                   operation_mode != OPERATION_RIGHT) {  // run right
          Serial.println(op);
          operation_mode = OPERATION_RIGHT;
          cheek.color(LEDCOLOR_GREEN);
          cheek.mode(LED_FADE);
          cheek.period(1);
          eyes.shape(SHAPE_ZITO);
          eyes.mode(EYE_MOVE);
          eyes.setMoveTarget(0, 1);
          arms.leftRatio(0);
          arms.rightRatio(100);
          head.tiltRatio(0);
        } else if (op == 's' && operation_mode != OPERATION_SPIN) {  // spin
          Serial.println(op);
          operation_mode = OPERATION_SPIN;
          cheek.color(LEDCOLOR_MAGENTA);
          cheek.mode(LED_FADE);
          cheek.period(1);
          eyes.shape(SHAPE_GURUGURU);
          eyes.mode(EYE_MOVE);
          eyes.setMoveTarget(-1, 0);
          arms.leftRatio(100);
          arms.rightRatio(100);
          head.tiltRatio(100);
        } else if (op == 'm' && operation_mode != OPERATION_MANUAL) {  // manual
          Serial.println(op);
          operation_mode = OPERATION_MANUAL;
          cheek.color(LEDCOLOR_BLUE);
          cheek.mode(LED_FADE);
          cheek.period(2);
          eyes.shape(SHAPE_NORMAL);
          eyes.mode(EYE_IDLE);
          arms.leftRatio(50);
          arms.rightRatio(50);
          head.setSlowTilt(0);
          move_timer = millis();
        } else if (op == '0' && operation_mode == OPERATION_MANUAL) {  // manual
          tilt.write(90-23);
        } else if (op == '1' && operation_mode == OPERATION_MANUAL) {  // manual
          tilt.write(90-25);
        } else if (op == '2' && operation_mode == OPERATION_MANUAL) {  // manual
          tilt.write(90-27);
        } else if (op == '3' && operation_mode == OPERATION_MANUAL) {  // manual
          tilt.write(90-29);
        } else if (op == '4' && operation_mode == OPERATION_MANUAL) {  // manual
          tilt.write(90-31);
        } else if (op == '5' && operation_mode == OPERATION_MANUAL) {  // manual
          tilt.write(90-33);
        } else if (op == '6' && operation_mode == OPERATION_MANUAL) {  // manual
          tilt.write(90-35);
        } else if (op == '7' && operation_mode == OPERATION_MANUAL) {  // manual
          tilt.write(90-37);
        } else if (op == '8' && operation_mode == OPERATION_MANUAL) {  // manual
          tilt.write(90-39);
        } else if (operation_mode == OPERATION_SLEEP) {
          operation_mode = OPERATION_MANUAL;
          cheek.color(LEDCOLOR_BLUE);
          cheek.mode(LED_FADE);
          cheek.period(2);
          eyes.shape(SHAPE_NORMAL);
          eyes.setMoveTarget(0, 0);
          eyes.mode(EYE_IDLE);
          arms.leftRatio(50);
          arms.rightRatio(50);
          head.setSlowTilt(0);
          move_timer = millis();
        }
      }
    }
  }

/* check environmental sound */
  int vol = getSoundLevel();
  //Serial.println(vol);
  if (vol > 0) {
    operation_timer = millis();
    if (operation_mode == OPERATION_SLEEP) {
      operation_mode = OPERATION_MANUAL;
      cheek.color(LEDCOLOR_BLUE);
      cheek.mode(LED_FADE);
      cheek.period(2);
      eyes.shape(SHAPE_NORMAL);
      eyes.mode(EYE_IDLE);
      arms.leftRatio(50);
      arms.rightRatio(50);
      head.setSlowTilt(0);
//      actions.queueHeadTilt(0);
//      actions.queueArmLeft(50, 50);
//      actions.queueArmRight(50, 50);
    }
  } 

  if (operation_mode == OPERATION_MANUAL) {
    /* enter sleep */
    if (millis() - operation_timer > 3 * 60 * 1000) {
      operation_mode = OPERATION_SLEEP;
      cheek.color(LEDCOLOR_CYAN);
      cheek.mode(LED_FADE);
      cheek.period(10);
      eyes.setMoveTarget(0, 0);
      eyes.shape(SHAPE_HORI);
      eyes.mode(EYE_SLEEP);
      arms.leftRatio(-50);
      arms.rightRatio(-50);
      head.setSlowTilt(-100);
//      actions.queueHeadTilt(-80);
//      actions.queueArmLeft(-50, 50);
//      actions.queueArmRight(-50, 50);
    } else {
      if (millis() - move_timer > 1000) {
        if (random(100) < 20) {
//          actions.queueHeadTilt(random(130) - 50);
          int ratio = 100 - random(200);
//          head.tiltRatio(ratio);
          head.setSlowTilt(ratio);
//          Serial.println(ratio);
//          Serial.println(head.tiltRatio());
        }
        move_timer = millis();
      }
    }
  }

  eyes.randomBlinking(operation_mode == OPERATION_MANUAL);
  eyes.blink();
  eyes.move();

  //actions.dequeue();

  cheek.update();
  if (head.updateTilt()) {
    Serial.println(tilt.read());
  }
/*
//  if (arms.updateLeft()) {
    // prefs.putShort("arm_left_deg", arms.left());
//    if (operation_mode == OPERATION_FORWARD ||
//        operation_mode == OPERATION_RIGHT) {
//      run_left = -run_left;
//      actions.queueArmLeft(run_left, 50);
    }
  }
  if (arms.updateRight()) {
    // prefs.putShort("arm_right_deg", arms.right());
    if (operation_mode == OPERATION_FORWARD ||
        operation_mode == OPERATION_LEFT) {
      run_right = -run_right;
      actions.queueArmRight(run_right, 50);
    }
  }
*/
  delay(1);
}

/*
void handleLed() {
  String argname, argv;
  int pos = 0, r = 0, g = 0, b = 0;
  for (int i = 0; i < webServer.args(); i++) {
    argname = webServer.argName(i);
    argv = webServer.arg(i);
    if (argname == "pos") {
      pos = argv.toInt();
    }
    if (argname == "r") {
      r = argv.toInt();
    }
    if (argname == "g") {
      g = argv.toInt();
    }
    if (argname == "b") {
      b = argv.toInt();
    }
  }
  leds.SetPixelColor(pos, RgbColor(r, g, b));
  leds.Show();
  handleStatus();
}

void handleArm() {
  String argname, argv;
  int d, s, t = 120, l = 200, r = 200;
  for (int i = 0; i < webServer.args(); i++) {
    argname = webServer.argName(i);
    argv = webServer.arg(i);
    if (argname == "left") {
      s = argv.toInt();
      l = constrain(s, -100, 100);
    } else if (argname == "right") {
      s = argv.toInt();
      r = constrain(s, -100, 100);
    } else if (argname == "time") {
      s = argv.toInt();
      t = constrain(s, 0, 10000);
    }
  }
  if (l < 200 && r < 200) {
    actions.queueArms(l, r, t);
  } else if (l < 200) {
    actions.queueArmLeft(l, t);
  } else if (r < 200) {
    actions.queueArmRight(r, t);
  }
  handleStatus();
}

void handleHead() {
  String argname, argv;
  int d, p = 200, t = 200;
  for (int i = 0; i < webServer.args(); i++) {
    argname = webServer.argName(i);
    argv = webServer.arg(i);
    if (argname == "pan") {
      p = argv.toInt();
      p = constrain(p, -100, 100);
      //      head.setSlowPan(p);
      //      servo1.write(90 - s);
    } else if (argname == "tilt") {
      t = argv.toInt();
      t = constrain(t, -100, 100);
      //      head.setSlowTilt(t);
      //      servo2.write(s + 90);
    }
  }
  if (p < 200 && t < 200) {
    actions.queueHead(p, t);
  } else if (p < 200) {
    actions.queueHeadPan(p);
  } else if (t < 200) {
    actions.queueHeadTilt(t);
  }
  handleStatus();
}

void handleEye() {
  String argname, argv;
  int d, s, left_shape, right_shape, mode = EYE_IDLE;
  left_shape = right_shape = SHAPE_NORMAL;
  for (int i = 0; i < webServer.args(); i++) {
    argname = webServer.argName(i);
    argv = webServer.arg(i);
    if (argname == "left") {
      left_shape = argv.toInt();
    } else if (argname == "right") {
      right_shape = argv.toInt();
    } else if (argname == "both") {
      left_shape = argv.toInt();
      right_shape = left_shape;
    } else if (argname == "wink") {
      mode = EYE_WINK;
    }
  }
  actions.queueEyes(left_shape, right_shape, mode);
  handleStatus();
}

void handleMode() {
  String argname, argv;
  int d;
  for (int i = 0; i < webServer.args(); i++) {
    argname = webServer.argName(i);
    argv = webServer.arg(i);
    if (argname == "manual") {
      operation_mode = OPERATION_MANUAL;
      eyes.shape(SHAPE_NORMAL);
      eyes.mode(EYE_IDLE);
      actions.queueHead(0, 0);
    } else if (argname == "automatic") {
      operation_mode = OPERATION_AUTO;
      eyes.shape(SHAPE_NORMAL);
      eyes.mode(EYE_IDLE);
      actions.queueHead(0, 0);
    } else if (argname == "sleep") {
      operation_mode = OPERATION_SLEEP;
      eyes.shape(SHAPE_HORI);
      eyes.mode(EYE_SLEEP);
      actions.queueHead(0, 80);
    }
  }
  prefs.putUInt("operation_mode", operation_mode);
  cheek.color(operation_mode == OPERATION_MANUAL
                  ? LEDCOLOR_RED
                  : (operation_mode == OPERATION_AUTO ? LEDCOLOR_MAGENTA
                                                      : LEDCOLOR_CYAN));
  cheek.mode(LED_FADE);
  cheek.period(operation_mode == OPERATION_SLEEP ? 10 : 3);
  handleStatus();
}

void handleBanzai() {
  String argname, argv;
  int num = 3;
  for (int i = 0; i < webServer.args(); i++) {
    argname = webServer.argName(i);
    argv = webServer.arg(i);
    if (argname == "repeat") {
      num = argv.toInt();
    }
  }
  actions.queueEyes(SHAPE_GT, SHAPE_LT, EYE_IDLE);
  actions.queueCheek(LED_ON, LEDCOLOR_RED);
  actions.queueHead(0, -100);
  for (int i = 0; i < num; i++) {
    actions.queueArms(-100, -100, 60);
    actions.queueWait(500);
    actions.queueArms(100, 100, 60);
    actions.queueWait(2000);
  }
  actions.queueArms(0, 0, 60);
  actions.queueHead(0, 0);
  actions.queueEyes(SHAPE_NORMAL, SHAPE_NORMAL, EYE_IDLE);
  handleStatus();
}

void handleConfig() {
  String argname, argv, message;
  int d, s;
  for (int i = 0; i < webServer.args(); i++) {
    argname = webServer.argName(i);
    argv = webServer.arg(i);
    if (argname == "hostname") {
      myhost = argv;
      prefs.putString("hostname", myhost);
    } else if (argname == "arm_left_min") {
      d = argv.toInt();
      prefs.putShort("arm_left_min_deg", d);
    } else if (argname == "arm_left_max") {
      d = argv.toInt();
      prefs.putShort("arm_left_max_deg", d);
    } else if (argname == "arm_right_min") {
      d = argv.toInt();
      prefs.putShort("arm_right_min_deg", d);
    } else if (argname == "arm_right_max") {
      d = argv.toInt();
      prefs.putShort("arm_right_max_deg", d);
    } else if (argname == "head_pan_min") {
      d = argv.toInt();
      prefs.putShort("head_pan_min_deg", d);
    } else if (argname == "head_pan_max") {
      d = argv.toInt();
      prefs.putShort("head_pan_max_deg", d);
    } else if (argname == "head_tilt_min") {
      d = argv.toInt();
      prefs.putShort("head_tilt_min_deg", d);
    } else if (argname == "head_tilt_max") {
      d = argv.toInt();
      prefs.putShort("head_tilt_max_deg", d);
    }
  }
  arms.setMinMax(prefs.getShort("arm_left_min_deg", LEFT_MIN_DEG),
                 prefs.getShort("arm_left_max_deg", LEFT_MAX_DEG),
                 prefs.getShort("arm_right_min_deg", RIGHT_MIN_DEG),
                 prefs.getShort("arm_right_max_deg", RIGHT_MAX_DEG));
  head.setMinMax(prefs.getShort("head_pan_min_deg", PAN_MIN_DEG),
                 prefs.getShort("head_pan_max_deg", PAN_MAX_DEG),
                 prefs.getShort("head_tilt_min_deg", TILT_MIN_DEG),
                 prefs.getShort("head_tilt_max_deg", TILT_MAX_DEG));
  DynamicJsonBuffer jsonBuffer;
  JsonObject &json = jsonBuffer.createObject();
  json["hostname"] = myhost;
  json["arm_left_min"] = prefs.getShort("arm_left_min_deg", LEFT_MIN_DEG);
  json["arm_left_max"] = prefs.getShort("arm_left_max_deg", LEFT_MAX_DEG);
  json["arm_right_min"] =
      prefs.getShort("arm_right_min_deg", RIGHT_MIN_DEG);
  json["arm_right_max"] =
      prefs.getShort("arm_right_max_deg", RIGHT_MAX_DEG);
  json["head_pan_min"] = prefs.getShort("head_pan_min_deg", PAN_MIN_DEG);
  json["head_pan_max"] = prefs.getShort("head_pan_max_deg", PAN_MAX_DEG);
  json["head_tilt_min"] =
      prefs.getShort("head_tilt_min_deg", TILT_MIN_DEG);
  json["head_tilt_max"] =
      prefs.getShort("head_tilt_max_deg", TILT_MAX_DEG);
  json.printTo(message);
  webServer.send(200, "application/json", message);
}

void handleReboot() {
  String message;
  DynamicJsonBuffer jsonBuffer;
  prefs.end();
  JsonObject &json = jsonBuffer.createObject();
  json["reboot"] = "done";
  json.printTo(message);
  webServer.send(200, "application/json", message);
  delay(5000);
  ESP.restart();
  while (1) {
    delay(0);
  }
}

String timestamp() {
  String ts;
  DateTime now = rtc.now();
  char str[20];
  sprintf(str, "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(),
          now.day(), now.hour(), now.minute(), now.second());
  ts = str;
  return ts;
}
*/