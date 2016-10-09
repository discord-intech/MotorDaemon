//
// Created by discord on 26/09/16.
//

#include "../include/Motor.hpp"


Motor::Motor(BlackLib::pwmName pwm, BlackLib::BlackGPIO dir1, BlackLib::BlackGPIO dir2) : PWMpin(pwm),
                                                                                          directionPin1(dir1),
                                                                                          directionPin2(dir2) {}

LeftMotor::LeftMotor() : Motor(BlackLib::EHRPWM1A,
                               BlackLib::BlackGPIO(BlackLib::GPIO_49, BlackLib::output, BlackLib::FastMode),
                               BlackLib::BlackGPIO(BlackLib::GPIO_60, BlackLib::output, BlackLib::FastMode)) {}

RightMotor::RightMotor() : Motor(BlackLib::EHRPWM1B,
                               BlackLib::BlackGPIO(BlackLib::GPIO_117, BlackLib::output, BlackLib::FastMode),
                               BlackLib::BlackGPIO(BlackLib::GPIO_125, BlackLib::output, BlackLib::FastMode)) {}

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

    pwm.setPeriodTime(PWM_TIME_PERIOD);
    pwm.setDutyPercent(0.0);
    pwm.setRunState(BlackLib::run);
}

void Motor::run(long duty) //duty € [-255;255]
{
    if(duty >= 0)
    {
        setDirection(Direction::FORWARD);
    }
    else
    {
        setDirection(Direction::BACKWARD);
    }

    pwm.setDutyCycle((uint64_t) ((ABS(duty) / 255) * PWM_TIME_PERIOD));
}
