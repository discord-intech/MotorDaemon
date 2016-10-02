//
// Created by discord on 30/09/16.
//

#include "../include/Servo.hpp"


Servo::Servo(float lowB, float lowA, float upB, float upA) : lowerBound(lowB), upperBound(upB), lowerAngle(lowA), upperAngle(upA)
{
    if(lowerBound >= upperBound || lowerAngle >= upperAngle)
    {
        throw std::invalid_argument("Servo.cpp : LowerBound/Angle is greater or equal to UpperBound/Angle");
    }
    PWMpin = BlackLib::EHRPWM2A;
}

void Servo::initPWM(void)
{
    pwm = BlackLib::BlackPWM(PWMpin);
    pwm.setPeriodTime(SERVO_PWM_TIME_PERIOD);
    pwm.setDutyPercent((float) ((upperBound - lowerBound) * 0.5 + lowerBound));
}

void Servo::setAngle(float angle)
{
    if(angle < lowerAngle || angle > upperAngle)
    {
        std::cout << "Servo.cpp : Bad angle received : " << angle << std::endl;
        return;
    }

    float duty = (angle-lowerAngle) / (upperAngle-lowerAngle);

    pwm.setDutyPercent(duty/10);
}
