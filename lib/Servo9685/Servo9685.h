#ifndef servo9685_h
#define servo9685_h

#include <Arduino.h>
#include <servo_PCA9685.h>

class Servo9685
{
public:
  Servo9685();
  uint8_t attach(int pin, servo_PCA9685 *servo);                             // attach the given pin to the next free channel, sets pinMode, returns channel number or 0 if failure
  uint8_t attach(int pin, servo_PCA9685 *servo, uint16_t min, uint16_t max); // as above but also sets min and max values for writes.
  void detach();
  void write(int value);             // if value is < 200 its treated as an angle, otherwise as pulse width in microseconds
  void writeMicroseconds(int value); // Write pulse width in microseconds
  int read();                        // returns current pulse width as an angle between 0 and 180 degrees
  int readMicroseconds();            // returns current pulse width in microseconds for this servo (was read_us() in first release)
  bool attached();                   // return true if this servo is attached, otherwise false
private:
  uint8_t _servoIndex; // index into the channel data for this servo
  uint16_t _minUs;
  uint16_t _maxUs;

  servo_PCA9685 *_servo;
};

#endif