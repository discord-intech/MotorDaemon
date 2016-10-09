//
// Created by discord on 30/09/16.
//

#ifndef MOTORDAEMON_SERVO_HPP
#define MOTORDAEMON_SERVO_HPP

#define SERVO_PWM_TIME_PERIOD 20  // millisecondes

#include "Motor.hpp"
#include <BlackPWM.h>

class Servo
{

public:
    Servo(float, float, float, float);
    void initPWM();

    void setAngle(float);

private:
    BlackLib::BlackPWM pwm = BlackLib::BlackPWM(BlackLib::PWMDISABLE);
    BlackLib::pwmName PWMpin;

    float lowerBound;
    float lowerAngle;
    float upperBound;
    float upperAngle;
};


#endif //MOTORDAEMON_SERVO_HPP
