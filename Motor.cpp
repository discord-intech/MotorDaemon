//
// Created by discord on 26/09/16.
//

#include "Motor.hpp"
#include <cmath>

void Motor::setDirection(Direction way)
{
    if(actualDirection == way)
        return;

    if(way == Direction.FORWARD)
    {
        //TODO
        if(side == Side.LEFT)
        {

        }
        else
        {

        }
    }
    else
    {
        //TODO
        if(side == Side.LEFT)
        {

        }
        else
        {

        }
    }

    actualDirection = way;

}

Motor::Motor(Side s) : side(s)
{
    //TODO activation des pins moteur selon le côté (gauche/droite) + marche avant par défaut
    PWMpin = BlackLib::EHRPWM0A; //placeholder
}

void Motor::initPWM()
{
    pwm = BlackLib::BlackPWM(PWMpin);

    pwm.setDutyPercent(0.0);
    pwm.setPeriodTime(PWM_TIME_PERIOD);
}

void Motor::run(int16_t duty) //duty € [-255;255]
{
    float percent = (float) ((MIN(std::abs(duty), 255.) / 255.) * 100);

    if(duty >= 0)
    {
        setDirection(Direction.FORWARD);
    }
    else
    {
        setDirection(Direction.BACKWARD);
    }

    pwm.setDutyPercent(percent);

}
