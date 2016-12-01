//
// Created by discord on 26/09/16.
//

#ifndef MOTORDAEMON_MOTOR_HPP
#define MOTORDAEMON_MOTOR_HPP

#include <cstdint>
#include <iostream>
#include <fstream>
#include <string>
#include "safe_enum.hpp"
//#include <BlackPWM.h>

#define MIN(x,y) (((x)<(y))?(x):(y))
#define MAX(x,y) (((x)>(y))?(x):(y))
#define ABS(x) (((x) > 0) ? (x) : -(x))
#define SIN(x) (1-(((x)*(x)*(x))/6))
#define COS(x) (1-(((x)*(x))/4))
#define ARCTAN(x) (1-(((x)*(x)*(x))/3))
#define TAN(x) ((x)+(((x)*(x)*(x))/3))

#define PWM_TIME_PERIOD 1000*1000  // nanosecondes

#define MINIMAL_PWM_PERC 0
#define MAXIMUM_PWM_PERC 1.0 //Used to limit PWM output

#define ECHO std::string("echo ")


    struct direction_def {
        enum type {
            BACKWARD, FORWARD
        };
    };

    typedef safe_enum<direction_def> Direction;

    class Motor {
    private:
       // BlackLib::BlackPWM pwm = BlackLib::BlackPWM(BlackLib::PWMDISABLE);
        uint8_t PWMpin;

        int directionPin;

        std::string PWMduty;

        FILE * dutyFile;

        bool inversed;

        Direction actualDirection;
        int actualDuty;
        void setDirection(Direction);
        void setDirectionPins(void);

        std::string dutyPath;

    public:
        Motor(uint8_t, int, bool);
        void initPWM(void);
        void run(int);
    };


    class LeftMotor : public Motor {
    public:
        LeftMotor();
    };

    class RightMotor : public Motor {
    public:
        RightMotor();
    };




#endif //MOTORDAEMON_MOTOR_HPP
