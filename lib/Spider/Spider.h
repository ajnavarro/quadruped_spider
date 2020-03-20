#ifndef spider_h
#define spider_h

#include <Arduino.h>
#include <Servo9685.h>

#define PI 3.1415926535897932384626433832795
#define KEEP 255

class Spider
{
public:
  Spider();
  void stand()
  {
    move_z(z_default);
  }
  void sit()
  {
    move_z(z_boot);
  }
  void turn_left(int steps);
  void turn_right(int steps);
  void step_forward(int steps);
  void step_backwards(int steps);
  void body_move(long x, long y, long z);
  void align();
  void servo_service();

private:
  void set_site(int leg, float x, float y, float z);
  void wait_reach(int leg);
  void wait_all_reach(void);
  void move_z(float z_pos);
  void servo_attach(void);

  void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z);
  void polar_to_servo(int leg, float alpha, float beta, float gamma);

  servo_PCA9685 *pwm;
  Servo9685 servo[4][3];
  const int servo_pin[4][3] = {{2, 3, 4}, {5, 6, 7}, {8, 9, 10}, {11, 12, 13}};
  /* Size of the robot ---------------------------------------------------------*/
  const float length_a = 55;
  const float length_b = 77.5;
  const float length_c = 27.5;
  const float length_side = 69.6;
  const float z_absolute = -31.6;
  /* Constants for movement ----------------------------------------------------*/
  const float z_default = -55, z_up = -35, z_boot = z_absolute;
  const float x_default = 60, x_offset = 0;
  const float y_start = 0, y_step = 45;
  const float y_default = x_default;
  /* variables for movement ----------------------------------------------------*/
  volatile float site_now[4][3];    //real-time coordinates of the end of each leg
  volatile float site_expect[4][3]; //expected coordinates of the end of each leg
  float temp_speed[4][3];           //each axis' speed, needs to be recalculated before each movement
  float move_speed;                 //movement speed
  float speed_multiple = 1;         //movement speed multiple
  const float spot_turn_speed = 4;
  const float leg_move_speed = 8;
  const float body_move_speed = 3;
  const float stand_seat_speed = 1;
  /* Constants for turn --------------------------------------------------------*/
  //temp length
  const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
  const float temp_b = 2 * (y_start + y_step) + length_side;
  const float temp_c = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2));
  const float temp_alpha = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);
  //site for turn
  const float turn_x1 = (temp_a - length_side) / 2;
  const float turn_y1 = y_start + y_step / 2;
  const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha);
  const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side;
};

#endif