//
// Created by discord on 30/09/16.
//

#include <stdlib.h>
#include "../include/Servo.hpp"


Servo::Servo(float lowB, float lowA, float upB, float upA) : lowerBound(lowB), upperBound(upB), lowerAngle(lowA), upperAngle(upA)
{
    if(lowerBound >= upperBound || lowerAngle >= upperAngle)
    {
        throw std::invalid_argument("Servo.cpp : LowerBound/Angle is greater or equal to UpperBound/Angle");
    }
    PWMduty = std::string(" > /sys/class/pwm/pwmchip6/pwm0/duty_cycle");

}

void Servo::initPWM(void)
{
    system((std::string("echo 0 > /sys/class/pwm/pwmchip6/export")).c_str());
    system((std::string("echo ")+std::to_string(SERVO_PWM_TIME_PERIOD)+std::string(" > /sys/class/pwm/pwmchip6/pwm0/period")).c_str());
    system((std::string("echo 0 > /sys/class/pwm/pwmchip6/pwm0/duty_cycle")).c_str());
    system((std::string("echo 1 > /sys/class/pwm/pwmchip6/pwm0/enable")).c_str());

    this->dutyPath = std::string("/sys/class/pwm/pwmchip6/pwm0/duty_cycle");

    this->dutyFile = fopen(this->dutyPath.c_str(), "w");

    if(this->dutyFile == NULL)
    {
        std::cerr << "INIT : Can't open duty file for PWM" << this->dutyPath << " !" << std::endl;
        return;
    }
}

void Servo::setAngle(float angle)
{
    /*if(angle < lowerAngle || angle > upperAngle)
    {
        std::cout << "Servo.cpp : Bad angle received : " << angle << std::endl;
        return;
    }*/

    this->dutyFile = freopen(dutyPath.c_str(), "w", this->dutyFile);

    if(this->dutyFile == NULL)
    {
        std::cout << "Can't open duty file for PWM" << dutyPath << " !" << std::endl;
        return;
    }

    //float duty = (angle-lowerAngle) / (upperAngle-lowerAngle);

    //pwm.setDutyPercent(duty);
    if(angle < 0) angle = 0;
    if(angle > 180) angle = 180;
    float max_ms(2.4), min_ms(0.5);
    int64_t value = (int64_t) (((max_ms - min_ms) / 180.0 * angle + min_ms) * 1000);
   // pwm.setPeriodTime(period, BlackLib::milisecond);
    fputs(std::to_string(value).c_str(), this->dutyFile);

    fflush(this->dutyFile);

}
