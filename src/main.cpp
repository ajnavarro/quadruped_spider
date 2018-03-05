/* -----------------------------------------------------------------------------
  - Project: Remote control Crawling robot
  - Author:  panerqiang@sunfounder.com
  - Date:  2015/1/27
   -----------------------------------------------------------------------------
  - Overview
  - This project was written for the Crawling robot desigened by Sunfounder.
    This version of the robot has 4 legs, and each leg is driven by 3 servos.
  This robot is driven by a Ardunio Nano Board with an expansion Board.
  We recommend that you view the product documentation before using.
  - Request
  - This project requires some library files, which you can find in the head of
    this file. Make sure you have installed these files.
  - How to
  - Before use,you must to adjust the robot,in order to make it more accurate.
    - Adjustment operation
    1.uncomment ADJUST, make and run
    2.comment ADJUST, uncomment VERIFY
    3.measure real sites and set to real_site[4][3], make and run
    4.comment VERIFY, make and run
  The document describes in detail how to operate.
   ---------------------------------------------------------------------------*/

// modified by Regis for spider project, 2015/09/11

/* Includes ------------------------------------------------------------------*/
#include <Servo9685.h> //to define and control servos
#include <servo_PCA9685.h>
#include <Ticker.h>
#include <Spider.h>

Ticker ticker;
Spider *spider;

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

    spider = new Spider();
    ticker.attach_ms(15, tick);

    Serial.println("Robot initialization Complete");
}

void loop()
{
    spider->stand();
    delay(2000);

    Serial.println("HEAD UP?");
    spider->head_move(-10);
    delay(2000);
    Serial.println("HEAD DOWN");
    spider->head_move(10);
    delay(2000);
    // Serial.println("Step forward");
    // spider->step_forward(5);
    // delay(2000);
    // Serial.println("Step backward");
    // spider->step_backwards(5);
    // delay(2000);
    // Serial.println("Turn left");
    // spider->turn_left(5);
    // delay(2000);
    // Serial.println("Turn right");
    // spider->turn_right(5);
    // delay(2000);
    // Serial.println("SIT");
    // spider->sit();
    // delay(2000);
    // spider->body_move(0, 0);
    // delay(500);

    // delay(500);
    // spider->body_move(6, -6);
    // delay(500);
    // spider->body_move(6, 6);
    // delay(500);
    // spider->body_move(-6, -6);
    // delay(500);
}
