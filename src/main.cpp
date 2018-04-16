#include <Servo9685.h>
#include <servo_PCA9685.h>
#include <Ticker.h>
#include <Spider.h>

#include <WiFiManager.h>

Ticker ticker;

Spider *spider = new Spider();

void tick()
{
  spider->servo_service();
}

void setup()
{
  //start serial for debug
  Serial.begin(115200);

  Serial.println("Robot starts initialization");
  //initialize default parameter

  ticker.attach_ms(15, tick);

  Serial.println("Robot initialization Complete");
}

void loop()
{
  spider->stand();
  delay(2000);
  spider->sit();
  delay(2000);
}
