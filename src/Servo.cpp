//
// Created by discord on 30/09/16.
//

#include "../include/Servo.hpp"


Servo::Servo(float lowB, float upB) : lowerBound(lowB), upperBound(upB)
{
    if(lowerBound >= upperBound)
    {
        throw std::invalid_argument("Servo.cpp : LowerBound is greater or equal to UpperBound");
    }
    PWMpin = BlackLib::EHRPWM2A;
}

void Servo::initPWM(void)
{
    pwm = BlackLib::BlackPWM(PWMpin);
    pwm.setPeriodTime(PWM_TIME_PERIOD);
    pwm.setDutyPercent((float) ((upperBound - lowerBound) * 0.5 + lowerBound));
}

void Servo::setAngle(float angle) //angle € [-255,255]
{
    float duty = (float) (((angle + 255.) / 511.) * (upperBound - lowerBound) + lowerBound); //TODO check that

    pwm.setDutyPercent(duty);
}
