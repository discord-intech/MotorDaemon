//
// Created by discord on 30/09/16.
//

#ifndef MOTORDAEMON_SERVO_HPP
#define MOTORDAEMON_SERVO_HPP

#include "Motor.hpp"

class Servo
{

public:
    Servo(float, float);
    void initPWM();

    void setAngle(float);

private:
    BlackLib::BlackPWM pwm = BlackLib::BlackPWM(BlackLib::PWMDISABLE);
    BlackLib::pwmName PWMpin;

    float lowerBound;
    float upperBound;
};


#endif //MOTORDAEMON_SERVO_HPP
