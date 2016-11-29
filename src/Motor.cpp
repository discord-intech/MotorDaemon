//
// Created by discord on 26/09/16.
//

#include <stdlib.h>
#include <iostream>
#include <cstring>
#include "../include/Motor.hpp"


Motor::Motor(uint8_t pwm, int dir, bool inv) : PWMpin(pwm), directionPin(dir), inversed(inv)
{
    system((std::string("echo ")+std::to_string(dir)+std::string(" > /sys/class/gpio/export")).c_str());

    system((std::string("echo out > /sys/class/gpio/gpio")+std::to_string(dir)+std::string("/direction")).c_str());

    PWMduty = std::string(" > /sys/class/pwm/pwmchip3/pwm")+std::to_string(PWMpin)+std::string("/duty_cycle");
}

LeftMotor::LeftMotor() : Motor(0, 49, true) {}

RightMotor::RightMotor() : Motor(1, 117, false) {}

void Motor::setDirection(Direction way)
{
    if (actualDirection == way)
    {
        return;
    }

    if(!inversed)
    {
        if (way == Direction::FORWARD) {
            system((std::string("echo 0 > /sys/class/gpio/gpio")+std::to_string(directionPin)+std::string("/value")).c_str());
        } else {
            system((std::string("echo 1 > /sys/class/gpio/gpio")+std::to_string(directionPin)+std::string("/value")).c_str());
        }
    }
    else
    {
        if (way == Direction::FORWARD) {
            system((std::string("echo 1 > /sys/class/gpio/gpio")+std::to_string(directionPin)+std::string("/value")).c_str());
        } else {
            system((std::string("echo 0 > /sys/class/gpio/gpio")+std::to_string(directionPin)+std::string("/value")).c_str());
        }
    }

    actualDirection = way;
}

void Motor::initPWM()
{
    //pwm = BlackLib::BlackPWM(PWMpin);

    setDirection(Direction::FORWARD);

    system((std::string("echo ")+std::to_string(PWMpin)+std::string(" > /sys/class/pwm/pwmchip3/export")).c_str());
    system((std::string("echo ")+std::to_string(PWM_TIME_PERIOD)+std::string(" > /sys/class/pwm/pwmchip3/pwm")+std::to_string(PWMpin)+std::string("/period")).c_str());
    system((std::string("echo 0 > /sys/class/pwm/pwmchip3/pwm")+std::to_string(PWMpin)+std::string("/duty_cycle")).c_str());
    system((std::string("echo 1 > /sys/class/pwm/pwmchip3/pwm")+std::to_string(PWMpin)+std::string("/enable")).c_str());


    //dutyFile.open((std::string("/sys/class/pwm/pwmchip3/pwm")+std::to_string(PWMpin)+std::string("/duty_cycle")).c_str(), std::ios::out);

    this->dutyPath = std::string("/sys/class/pwm/pwmchip3/pwm") + std::to_string(PWMpin) + std::string("/duty_cycle");

    this->dutyFile = fopen(this->dutyPath.c_str(), "w");

    if(this->dutyFile == NULL)
    {
        std::cerr << "INIT : Can't open duty file for PWM" << this->dutyPath << " !" << std::endl;
        return;
    }

   /* if(dutyFile == NULL)
    {
        std::cout << "Can't open duty file for PWM" << PWMpin << " !" << std::endl;
    }*/

  /*  pwm.setPeriodTime(PWM_TIME_PERIOD);
    pwm.setDutyPercent(0.0);
    pwm.setRunState(BlackLib::run);*/
}

void Motor::run(int duty) //duty € [-255;255]
{
    if(duty == this->actualDuty)
    {
        return;
    }

    if(duty >= 0)
    {
        setDirection(Direction::FORWARD);
    }
    else
    {
        setDirection(Direction::BACKWARD);
    }
    //system((ECHO+std::to_string((ABS(duty) / 255) * PWM_TIME_PERIOD)+PWMduty).c_str());

    this->dutyFile = freopen(dutyPath.c_str(), "w", this->dutyFile);

    if(this->dutyFile == NULL)
    {
        std::cout << "Can't open duty file for PWM" << dutyPath << " !" << std::endl;
        return;
    }

    if(ABS(duty) > 10)
    {
        fputs(std::to_string((int)(((float)ABS(duty) / 255.) * PWM_TIME_PERIOD * LIMITER)).c_str(), this->dutyFile);
    }
    else
    {
        fputs(std::to_string(0).c_str(), this->dutyFile);
    }

    fflush(this->dutyFile);

    //dutyFile.seekp(std::ios::beg);
    //dutyFile << (int)(((float)ABS(duty) / 255.) * PWM_TIME_PERIOD);
    //dutyFile.flush();

    // fseek (dutyFile, 0, SEEK_SET);
    //fclose(dutyFile);
    this->actualDuty = duty;
    //pwm.setDutyCycle((uint64_t) ((ABS(duty) / 255) * PWM_TIME_PERIOD));
}
