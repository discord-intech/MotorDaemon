//
// Created by discord on 26/09/16.
//

#include "Motor.hpp"


Motor::Motor(Side s) : side(s)
{
    if(side == Side::LEFT)
    {
        PWMpin = BlackLib::EHRPWM1A; //placeholder
        directionPin1 = BlackLib::BlackGPIO(BlackLib::GPIO_48, BlackLib::output, BlackLib::SecureMode);
        directionPin2 = BlackLib::BlackGPIO(BlackLib::GPIO_49, BlackLib::output, BlackLib::SecureMode);
    } else {
        PWMpin = BlackLib::EHRPWM1B;
        directionPin1 = BlackLib::BlackGPIO(BlackLib::GPIO_117, BlackLib::output, BlackLib::SecureMode);
        directionPin2 = BlackLib::BlackGPIO(BlackLib::GPIO_125, BlackLib::output, BlackLib::SecureMode);
    }
}

void Motor::setDirection(Direction way)
{
    if (actualDirection == way)
        return;

    //TODO Check side
    if (way == Direction::FORWARD) {
        directionPin1.setValue(BlackLib::high);
        directionPin2.setValue(BlackLib::low);

    } else {
        directionPin1.setValue(BlackLib::low);
        directionPin2.setValue(BlackLib::high);
    }

    actualDirection = way;
}

void Motor::initPWM()
{
    pwm = BlackLib::BlackPWM(PWMpin);

    setDirection(Direction::FORWARD);

    pwm.setDutyPercent(0.0);
    pwm.setPeriodTime(PWM_TIME_PERIOD);
}

void Motor::run(int16_t duty) //duty € [-255;255]
{
    float percent = (float) ((MIN(ABS(duty), 255.) / 255.) * 100);

    if(duty >= 0)
    {
        setDirection(Direction::FORWARD);
    }
    else
    {
        setDirection(Direction::BACKWARD);
    }

    pwm.setDutyPercent(percent);
}
