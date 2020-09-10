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
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <ESP32Servo.h>
#include <NeoPixelBus.h>
#include <Preferences.h>
#include <RTClib.h>
#include <SPI.h>
#include <SparkFun_APDS9960.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <Wire.h>

#include "actionqueue.h"
#include "aquatan_arms.h"
#include "aquatan_eye.h"
#include "big_aquatan.h"
#include "camera.h"
#include "neopixels.h"
#include "ntp.h"

#define PIN_SDA1 (16)
#define PIN_SCL1 (17)
#define PIN_NEOPIXEL (5)
#define PIN_SERVO1 (18)
#define PIN_SERVO2 (19)
#define PIN_SERVO3 (23)
#define PIN_SERVO4 (26)

#define PIN_MIC (36)

#define NEOPIXEL_LED_NUM (2)

Adafruit_SSD1306 oled1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 oled2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);

SparkFun_APDS9960 apds = SparkFun_APDS9960(&Wire1);

AquatanEye eye_left(&oled1);
AquatanEye eye_right(&oled2);
AquatanEyes eyes(&eye_left, &eye_right);

Servo pan, tilt, arm_left, arm_right;
Camera head(&pan, &tilt);
AquatanArms arms(&arm_left, &arm_right);

NeoPixelBus<NeoRgbFeature, Neo800KbpsMethod> leds(NEOPIXEL_LED_NUM,
                                                  PIN_NEOPIXEL);
NeoPixels cheek(&leds);

actionQueue actions(&eyes, &arms, &head, &cheek);

WebServer webServer(80);
NTP ntp("ntp.nict.jp");
RTC_Millis rtc;

Preferences prefs;

String myhost;
int operation_mode;

uint32_t prev_time;
uint32_t prev_frame;

uint32_t voice_time;
uint32_t operation_timer;


enum { OPERATION_AUTO = 0, OPERATION_MANUAL, OPERATION_SLEEP };

void setup() {
  WiFiManager wifiManager;

  Serial.begin(115200);
  rtc.begin(DateTime(2019, 12, 15, 15, 45, 0));
  prefs.begin("aquatan", false);

  myhost = prefs.getString("hostname", "bigaquatan");
  operation_mode = prefs.getUInt("operation_mode", OPERATION_MANUAL);

  leds.Begin();

  cheek.addPixel(0);
  cheek.addPixel(1);
  cheek.mode(LED_OFF);
  cheek.update();
  cheek.color(operation_mode == OPERATION_MANUAL
                  ? LEDCOLOR_RED
                  : (operation_mode == OPERATION_AUTO ? LEDCOLOR_MAGENTA
                                                      : LEDCOLOR_CYAN));
  cheek.period(operation_mode == OPERATION_SLEEP ? 10 : 3);
  cheek.mode(LED_FADE);

  head.begin(prefs.getShort("pan_deg", 0), prefs.getShort("tilt_deg", 0));
  head.setMinMax(prefs.getShort("head_pan_min_deg", PAN_MIN_DEG),
                 prefs.getShort("head_pan_max_deg", PAN_MAX_DEG),
                 prefs.getShort("head_tilt_min_deg", TILT_MIN_DEG),
                 prefs.getShort("head_tilt_max_deg", TILT_MAX_DEG));
  arms.begin(prefs.getShort("arm_left_deg", 0),
             prefs.getShort("arm_right_deg", 0));
  arms.setMinMax(prefs.getShort("arm_left_min_deg", LEFT_MIN_DEG),
                 prefs.getShort("arm_left_max_deg", LEFT_MAX_DEG),
                 prefs.getShort("arm_right_min_deg", RIGHT_MIN_DEG),
                 prefs.getShort("arm_right_max_deg", RIGHT_MAX_DEG));

  Wire1.begin(PIN_SDA1, PIN_SCL1);

  eyes.begin(-5, 0, 0, 0);

  if (apds.init()) {
    Serial.println(F("LAPDS-9960 initialization complete"));
  } else {
    Serial.println(F("Something went wrong during APDS-9960 init!"));
  }
  if (apds.enableGestureSensor(true)) {
    Serial.println(F("Gesture sensor is now running"));
  } else {
    Serial.println(F("Something went wrong during gesture sensor init!"));
  }

  wifiManager.autoConnect("AQUATAN4");
  ArduinoOTA.setHostname(myhost.c_str());

  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS
        // using SPIFFS.end()
        Serial.println("Start updating " + type);
        pinMode(2, OUTPUT);
      })
      .onEnd([]() {
        digitalWrite(2, LOW);
        Serial.println("\nEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        digitalWrite(2, (progress / (total / 100)) % 2);
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
          Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
          Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
          Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
          Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
          Serial.println("End Failed");
      });
  ArduinoOTA.begin();

  webServer.on("/", handleStatus);
  webServer.on("/reboot", handleReboot);
  webServer.on("/status", handleStatus);
  webServer.on("/rawstatus", handleRawStatus);
  webServer.on("/config", handleConfig);
  webServer.on("/head", handleHead);
  webServer.on("/led", handleLed);
  webServer.on("/eye", handleEye);
  webServer.on("/arm", handleArm);
  webServer.on("/mode", handleMode);
  webServer.on("/banzai", handleBanzai);

  ntp.begin();
  webServer.begin();
  uint32_t epoch = ntp.getTime();
  if (epoch > 0) {
    rtc.adjust(DateTime(epoch + SECONDS_UTC_TO_JST));
  }
  operation_timer = millis();
}

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

void loop() {
  webServer.handleClient();
  ArduinoOTA.handle();
  DateTime now = rtc.now();

  /* random head move */
  if (millis() - prev_time > 1000) {
    if (operation_mode == OPERATION_AUTO) {
      if (random(100) < 10) {
        int r1 = random(-100, 100);
        int r2 = random(-100, 100);
        actions.queueHead(r1, r2);
        //        head.setSlowPan(r1);
        //        head.setSlowTilt(r2);
        int r3 = random(-100, 100);
        int r4 = random(-100, 100);
        actions.queueArms(r3, r4, 60);
        //        arms.setMoveLeft(r3,30);
        //        arms.setMoveRight(r4,30);
      }
    }
    prev_time = millis();
  }

  /* enter sleep */
  if (operation_mode == OPERATION_MANUAL || operation_mode == OPERATION_AUTO) {
    if (millis() - operation_timer > 1 * 60 * 1000) {
      operation_mode = OPERATION_SLEEP;
      cheek.color(LEDCOLOR_CYAN);
      cheek.mode(LED_FADE);
      cheek.period(10);
      eyes.shape(SHAPE_HORI);
      eyes.mode(EYE_SLEEP);
      actions.queueHead(0, 80);
    }
  }

  /* check environmental sound */
  int vol = getSoundLevel();
  //Serial.println(vol);
  if (vol > 0) {
    operation_timer = millis();
    if (operation_mode == OPERATION_SLEEP) {
      operation_mode = OPERATION_MANUAL;
      cheek.color(LEDCOLOR_RED);
      cheek.mode(LED_FADE);
      cheek.period(3);
      eyes.shape(SHAPE_NORMAL);
      eyes.mode(EYE_IDLE);
      actions.queueHead(0, 0);
    }
  } 

  /* check gestures */
  handleGesture();

  eyes.randomBlinking();

  eyes.blink();
  eyes.move();

  actions.dequeue();

  cheek.update();
  if (head.updatePan()) {
    prefs.putShort("pan_deg", head.pan());
  }
  if (head.updateTilt()) {
    prefs.putShort("tilt_deg", head.tilt());
  }
  if (arms.updateLeft()) {
    prefs.putShort("arm_left_deg", arms.left());
  }
  if (arms.updateRight()) {
    prefs.putShort("arm_right_deg", arms.right());
  }

  delay(1);
}

void handleGesture() {
  static uint32_t prev_millis = 0;
  if (operation_mode == OPERATION_MANUAL) {
    if (millis() - prev_millis > 100) {
      prev_millis = millis();
      if (apds.isGestureAvailable()) {
        operation_timer = millis();
        Serial.print("gesture avairable: ");
        int g = apds.readGesture(); // ここで詰まる
                                    //      Serial.println(g);
        switch (g) {
        case DIR_UP:
          Serial.println("UP");
          actions.queueHead(head.panRatio(), -100);
          // digitalWrite(LED_L, LOW);
          // Keyboard.press(KEY_UP_ARROW);
          // count++;
          break;
        case DIR_DOWN:
          Serial.println("DOWN");
          actions.queueHead(head.panRatio(), 80);
          // digitalWrite(LED_R, LOW);
          // Keyboard.press(KEY_DOWN_ARROW);
          // count++;
          break;
        case DIR_LEFT:
          Serial.println("LEFT");
          actions.queueHead(80, head.tiltRatio());
          // digitalWrite(LED_L, LOW);
          // Keyboard.press(KEY_LEFT_ARROW);
          // count++;
          break;
        case DIR_RIGHT:
          Serial.println("RIGHT");
          actions.queueHead(-80, head.tiltRatio());
          // digitalWrite(LED_R, LOW);
          // Keyboard.press(KEY_RIGHT_ARROW);
          // count++;
          break;
        case DIR_NEAR:
          Serial.println("NEAR");
          // digitalWrite(LED_R, LOW);
          // digitalWrite(LED_L, LOW);
          // Keyboard.write('h');
          // count++;
          break;
        case DIR_FAR:
          Serial.println("FAR");
          break;
        default:
          // Serial.println("NONE");
          break;
        }
      }
    }
  }
}

void handleStatus() {
  String message;
  DynamicJsonBuffer jsonBuffer;
  JsonObject &json = jsonBuffer.createObject();
  JsonObject &eyesobj = json.createNestedObject("eyes");
  eyesobj["left"] = eye_left.shape();
  eyesobj["right"] = eye_right.shape();
  JsonObject &camobj = json.createNestedObject("head");
  camobj["pan"] = head.pan();
  camobj["tilt"] = head.tilt();
  camobj["pan_ratio"] = head.panRatio();
  camobj["tilt_ratio"] = head.tiltRatio();
  JsonObject &armobj = json.createNestedObject("arm");
  armobj["left"] = arms.left();
  armobj["right"] = arms.right();
  armobj["left_ratio"] = arms.leftRatio();
  armobj["right_ratio"] = arms.rightRatio();
  //  json["mode"] = opmode[operation_mode];
  json["timestamp"] = timestamp();
  json.printTo(message);
  webServer.send(200, "application/json", message);
}

void handleRawStatus() {
  String message;
  DynamicJsonBuffer jsonBuffer;
  JsonObject &json = jsonBuffer.createObject();
  json["eye_left_mode"] = eye_left.mode();
  json["eye_right_mode"] = eye_right.mode();
  json["eye_left_blink"] = eye_left.blink_st();
  json["eye_right_blink"] = eye_right.blink_st();
  json["servo1"] = pan.read();
  json["servo2"] = arm_left.read();
  json["servo3"] = tilt.read();
  json["servo4"] = arm_right.read();
  json["timestamp"] = timestamp();
  json.printTo(message);
  webServer.send(200, "application/json", message);
}

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
