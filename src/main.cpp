#include <Servo9685.h>
#include <servo_PCA9685.h>
#include <SpiderV2.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "RemoteDebug.h"

#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>

#define HOSTNAME "spider"

SpiderV2 spider;

RemoteDebug Debug;

void setupWM()
{
  WiFiManager wifiManager;
  if (!wifiManager.autoConnect(HOSTNAME))
  {
    Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  }
}

void setupOTA()
{
  ArduinoOTA.setHostname(HOSTNAME);

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
    {
      type = "sketch";
    }
    else
    { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
    {
      Serial.println("Auth Failed");
    }
    else if (error == OTA_BEGIN_ERROR)
    {
      Serial.println("Begin Failed");
    }
    else if (error == OTA_CONNECT_ERROR)
    {
      Serial.println("Connect Failed");
    }
    else if (error == OTA_RECEIVE_ERROR)
    {
      Serial.println("Receive Failed");
    }
    else if (error == OTA_END_ERROR)
    {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
}

void processCmdRemoteDebug()
{

  String lastCmd = Debug.getLastCommand();

  if (lastCmd == "sit")
  {
    spider.sit();
    debugI("SIT");
  }

  if (lastCmd == "stand")
  {
    spider.stand();
    debugI("STAND");
  }

  debugI("STEP %d", spider.step);
  debugI("ACTUAL COMMAND %d", spider.actual_command);
  debugI("NEXT COMMAND %d", spider.next_command);
}

void setup()
{
  //start serial for debug
  Serial.begin(115200);
  Serial.println("Robot starts initialization");

  setupWM();

  // Initialize RemoteDebug
  Debug.begin(HOSTNAME); // Initialize the WiFi server

  //Debug.setPassword("r3m0t0."); // Password for WiFi client connection (telnet or webapp)  ?

  Debug.setResetCmdEnabled(true); // Enable the reset command
  Debug.showProfiler(true);       // Profiler (Good to measure times, to optimize codes)
  Debug.showColors(true);         // Colors

  // Debug.setSerialEnabled(true); // if you wants serial echo - only recommended if ESP is plugged in USB

  Debug.setCallBackProjectCmds(&processCmdRemoteDebug);

  setupOTA();

  Serial.println("Robot initialization Complete");
}

void loop()
{
  spider.loop();
  Debug.handle();
  ArduinoOTA.handle();

  String lastCmd = Debug.getLastCommand();

  if (lastCmd == "fo")
  {
    spider.forward();
  }

  if (lastCmd == "ba")
  {
    spider.backward();
  }

  if (lastCmd == "le")
  {
    spider.left();
  }

  if (lastCmd == "ri")
  {
    spider.right();
  }

  if (lastCmd == "st")
  {
    spider.stop();
  }
}