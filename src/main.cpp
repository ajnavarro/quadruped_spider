#include <Servo9685.h>
#include <servo_PCA9685.h>
#include <Ticker.h>
#include <Spider.h>
#include <JeVe_EasyOTA.h>
#include "RemoteDebug.h"

#ifndef STASSID
#define HOSTNAME "spider"
#define STASSID "wifi"
#define STAPSK "xxx"
#endif

const char *ssid = STASSID;
const char *password = STAPSK;

Ticker ticker;

Spider *spider = new Spider();

RemoteDebug Debug;
EasyOTA OTA(HOSTNAME);

void tick()
{
  spider->servo_service();
}

void setup()
{
  //start serial for debug
  Serial.begin(115200);
  Serial.println("Robot starts initialization");

  // Initialize RemoteDebug
  Debug.begin(HOSTNAME); // Initialize the WiFi server

  //Debug.setPassword("r3m0t0."); // Password for WiFi client connection (telnet or webapp)  ?

  Debug.setResetCmdEnabled(true); // Enable the reset command
  Debug.showProfiler(true);       // Profiler (Good to measure times, to optimize codes)
  Debug.showColors(true);         // Colors

  // Debug.setSerialEnabled(true); // if you wants serial echo - only recommended if ESP is plugged in USB

  // Project commands
  OTA.onMessage([](const String &message, int line) {
    Serial.println(message);
  });
  OTA.addAP(STASSID, STAPSK);

  Serial.println("Robot initialization Complete");

  spider->sit();
}

void loop()
{
  Debug.handle();
  OTA.loop();

  spider->step_forward(5);
  spider->turn_left(4);
  spider->step_forward(5);
  spider->turn_left(4);
  spider->step_forward(5);
  spider->turn_left(4);
  spider->step_forward(5);
  spider->turn_left(4);
}