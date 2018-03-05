#include "Servo9685.h"
#include <Wire.h>
#include <servo_PCA9685.h>

// similiar to map but will have increased accuracy that provides a more
// symetric api (call it and use result to reverse will provide the original value)
//
int improved_map(int value, int minIn, int maxIn, int minOut, int maxOut)
{
    const int rangeIn = maxIn - minIn;
    const int rangeOut = maxOut - minOut;
    const int deltaIn = value - minIn;
    // fixed point math constants to improve accurancy of divide and rounding
    const int fixedHalfDecimal = 1;
    const int fixedDecimal = fixedHalfDecimal * 2;

    return ((deltaIn * rangeOut * fixedDecimal) / (rangeIn) + fixedHalfDecimal) / fixedDecimal + minOut;
}

Servo9685::Servo9685()
{
}

uint8_t Servo9685::attach(int pin, servo_PCA9685 *servo)
{
    _servo = servo;
    _servoIndex = pin;
}

uint8_t Servo9685::attach(int pin, servo_PCA9685 *servo, uint16_t min, uint16_t max)
{
    _servo = servo;
    _servoIndex = pin;

    _servo->setServoMin(_servoIndex, min);
    _servo->setServoMax(_servoIndex, max);
}

void Servo9685::detach()
{
}

void Servo9685::write(int value)
{
    value = constrain(value, 0, 180);
    // writeMicroseconds will contrain the calculated value for us
    // for any user defined min and max, but we must use default min max
    value = improved_map(value, 0, 180, _servo->getServoMin(_servoIndex), _servo->getServoMax(_servoIndex));

    writeMicroseconds(value);
}

void Servo9685::writeMicroseconds(int value)
{
    _servo->setPWM(_servoIndex, 0, value);
}

int Servo9685::read()
{
    return 0;
}

int Servo9685::readMicroseconds()
{
    return 0;
}

bool Servo9685::attached()
{
    return true;
}
