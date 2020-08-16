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

#define NEOPIXEL_LED_NUM (2)

Adafruit_SSD1306 oled1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 oled2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);

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

enum { OPERATION_AUTO = 0, OPERATION_MANUAL };

void setup() {
  WiFiManager wifiManager;

  Serial.begin(115200);
  rtc.begin(DateTime(2019, 5, 1, 0, 0, 0));
  prefs.begin("aquatan", false);

  myhost = prefs.getString("hostname", "bigaquatan");
  operation_mode = prefs.getUInt("operation_mode", OPERATION_MANUAL);

  leds.Begin();

  cheek.addPixel(0);
  cheek.addPixel(1);
  cheek.mode(LED_OFF);
  cheek.update();
  cheek.color(operation_mode ? LEDCOLOR_CYAN : LEDCOLOR_MAGENTA);
  cheek.period(5);
  cheek.mode(LED_FADE);

  head.begin(prefs.getShort("pan_deg", 0), prefs.getShort("tilt_deg", 0));
  arms.begin(prefs.getShort("arm_left_deg", 0),
             prefs.getShort("arm_right_deg", 0));

  Wire1.begin(PIN_SDA1, PIN_SCL1);

  eyes.begin(-5, 0, 0, 0);

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

  ntp.begin();
  webServer.begin();
  uint32_t epoch = ntp.getTime();
  if (epoch > 0) {
    rtc.adjust(DateTime(epoch + SECONDS_UTC_TO_JST));
  }
}

uint32_t prev_time;
uint32_t prev_frame;

void loop() {
  webServer.handleClient();
  ArduinoOTA.handle();
  DateTime now = rtc.now();

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
      head.setSlowPan(p);
      //      servo1.write(90 - s);
    } else if (argname == "tilt") {
      t = argv.toInt();
      t = constrain(t, -100, 100);
      head.setSlowTilt(t);
      //      servo2.write(s + 90);
    }
  }
  if (p < 200 && t < 200) {
    actions.queueHead(p,t);
  } else if (p < 200) {
    actions.queueHeadPan(p);
  } else if (t < 200) {
    actions.queueHeadTilt(t);
  }
  handleStatus();
}

void handleEye() {
  String argname, argv;
  int d, s, left_shape, right_shape;
  left_shape = right_shape = 0;
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
      eyes.mode(EYE_WINK);
    }
  }
  eye_left.shape(left_shape);
  eye_right.shape(right_shape);
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
    } else if (argname == "automatic") {
      operation_mode = OPERATION_AUTO;
    }
  }
  prefs.putUInt("operation_mode", operation_mode);
  cheek.color(operation_mode == OPERATION_MANUAL ? LEDCOLOR_CYAN
                                                 : LEDCOLOR_MAGENTA);
  cheek.mode(LED_FADE);
  cheek.period(5);
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
    }
    /*    if (argname == "dburl") {
          dburl = argv;
          prefs.putString("dburl", dburl);
        }
    */
    //     else if (argname == "maxactivetime") {
    //      max_active_time = argv.toInt();
    //      prefs.putUInt("max_active_time", max_active_time);
    //    }
  }
  DynamicJsonBuffer jsonBuffer;
  JsonObject &json = jsonBuffer.createObject();
  json["hostname"] = myhost;
  //  json["powerhost"] = powerhost;
  //  json["max_active_time"] = max_active_time;
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
