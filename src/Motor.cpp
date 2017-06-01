//
// Created by discord on 26/09/16.
//

#include <stdlib.h>
#include <iostream>
#include <cstring>
#include "../include/Motor.hpp"


Motor::Motor(uint8_t pwm, int dir1, int dir2, bool inv, Settings &s) : PWMpin(pwm), directionPin(dir1), directionPin2(dir2),
                                                                       inversed(inv), settings(s), actualDirection(Direction::BACKWARD)
{
    system((std::string("echo ")+std::to_string(dir1)+std::string(" > /sys/class/gpio/export")).c_str());

    system((std::string("echo out > /sys/class/gpio/gpio")+std::to_string(dir1)+std::string("/direction")).c_str());

    system((std::string("echo ")+std::to_string(dir2)+std::string(" > /sys/class/gpio/export")).c_str());

    system((std::string("echo out > /sys/class/gpio/gpio")+std::to_string(dir2)+std::string("/direction")).c_str());

    PWMduty = std::string(" > /sys/class/pwm/pwmchip3/pwm")+std::to_string(PWMpin)+std::string("/duty_cycle");
}

LeftMotor::LeftMotor(Settings &s) : Motor(0, s.getInt("DIRECTION_PIN1_L"), s.getInt("DIRECTION_PIN2_L"),  false, s) {}

RightMotor::RightMotor(Settings &s) : Motor(1, s.getInt("DIRECTION_PIN1_R"), s.getInt("DIRECTION_PIN2_R"), true, s) {}

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
            system((std::string("echo 1 > /sys/class/gpio/gpio")+std::to_string(directionPin2)+std::string("/value")).c_str());
        } else {
            system((std::string("echo 1 > /sys/class/gpio/gpio")+std::to_string(directionPin)+std::string("/value")).c_str());
            system((std::string("echo 0 > /sys/class/gpio/gpio")+std::to_string(directionPin2)+std::string("/value")).c_str());
        }
    }
    else
    {
        if (way == Direction::FORWARD) {
            system((std::string("echo 1 > /sys/class/gpio/gpio")+std::to_string(directionPin)+std::string("/value")).c_str());
            system((std::string("echo 0 > /sys/class/gpio/gpio")+std::to_string(directionPin2)+std::string("/value")).c_str());
        } else {
            system((std::string("echo 0 > /sys/class/gpio/gpio")+std::to_string(directionPin)+std::string("/value")).c_str());
            system((std::string("echo 1 > /sys/class/gpio/gpio")+std::to_string(directionPin2)+std::string("/value")).c_str());
        }
    }

    actualDirection = way;
}

void Motor::initPWM()
{
    //pwm = BlackLib::BlackPWM(PWMpin);

    setDirection(Direction::FORWARD);

    system((std::string("echo ")+std::to_string(PWMpin)+std::string(" > /sys/class/pwm/pwmchip3/export")).c_str());
    system((std::string("echo ")+std::to_string((int)PWM_TIME_PERIOD)+std::string(" > /sys/class/pwm/pwmchip3/pwm")+std::to_string(PWMpin)+std::string("/period")).c_str());
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

    create_itoa_lookup_table();

    std::cout << "Printing itoa table :" << std::endl;
    for(int i =0 ; i<256 ; i+=5)
    {
        std::cout << i << " : " << itoa_lookup_table[i] << std::endl;
    }

   /* if(dutyFile == NULL)
    {
        std::cout << "Can't open duty file for PWM" << PWMpin << " !" << std::endl;
    }*/

  /*  pwm.setPeriodTime(PWM_TIME_PERIOD);
    pwm.setDutyPercent(0.0);
    pwm.setRunState(BlackLib::run);*/
}

void Motor::create_itoa_lookup_table(void)
{
    std::cout << "Creating ITOA table" << std::endl;

    for(int i=0 ; i<256 ; i++)
    {
        this->itoa_lookup_table[i] = std::to_string((long)(((double)i / 255.) * MAXIMUM_PWM_PERC * PWM_TIME_PERIOD));
    }
}

void Motor::run(int duty) //duty € [-255;255]
{

    if(duty == this->actualDuty)
    {
        return;
    }

    if(duty < -255)
    {
        duty = -255;
    }

    if(duty > 255)
    {
        duty = 255;
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

    if(ABS(duty) > 255*MINIMAL_PWM_PERC)
    {
        fputs(itoa_lookup_table[ABS(duty)].c_str(), this->dutyFile);
    }
    else
    {
        fputs(itoa_lookup_table[0].c_str(), this->dutyFile);
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
