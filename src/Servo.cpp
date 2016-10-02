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
    pwm.setPeriodTime(SERVO_PWM_TIME_PERIOD, BlackLib::milisecond);
    pwm.setDutyPercent((float) ((upperBound - lowerBound) * 0.5 + lowerBound));
    pwm.setRunState(BlackLib::run);
}

void Servo::setAngle(float angle)
{
   /* if(angle < lowerAngle || angle > upperAngle)
    {
        std::cout << "Servo.cpp : Bad angle received : " << angle << std::endl;
        return;
    }

    float duty = (angle-lowerAngle) / (upperAngle-lowerAngle);

    pwm.setDutyPercent(duty);*/
    if(angle < 0) angle = 0;
    if(angle > 180) angle = 180;
    float max_ms(2.4), min_ms(0.5), pwmFreq(50), period(1000.0 / pwmFreq);
    int64_t value = ((max_ms - min_ms) / 180.0 * angle + min_ms) * 1000;
    pwm.setPeriodTime(period, BlackLib::milisecond);
    pwm.setLoadRatioTime(value, BlackLib::microsecond);
}
