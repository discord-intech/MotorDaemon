//
// Created by discord on 26/09/16.
//

#ifndef MOTORDAEMON_MOTOR_HPP
#define MOTORDAEMON_MOTOR_HPP

#include <cstdint>
#include <string.h>
#include <iostream>
#include <fstream>
#include <string>
#include "safe_enum.hpp"
#include "Settings.hpp"
//#include <BlackPWM.h>

#define MIN(x,y) (((x)<(y))?(x):(y))
#define MAX(x,y) (((x)>(y))?(x):(y))
#define ABS(x) (((x) > 0) ? (x) : -(x))
#define SIN(x) (1-(((x)*(x)*(x))/6.0))
#define COS(x) (1-(((x)*(x))/4.0))
#define ARCTAN(x) ((x)-(((x)*(x)*(x))/3.0))
#define TAN(x) ((x)+(((x)*(x)*(x))/3.0))

#define PWM_TIME_PERIOD (1000.0*1000.0)  // nanosecondes

#define MINIMAL_PWM_PERC (settings.getFloat("MIN_PWM_PERC"))
#define MAXIMUM_PWM_PERC (settings.getFloat("MAX_PWM_PERC")) //Used to limit PWM output

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

        Settings settings;

        bool inversed;

        Direction actualDirection;
        int actualDuty;
        void setDirection(Direction);
        void setDirectionPins(void);

        void create_itoa_lookup_table(void);

        std::string itoa_lookup_table[256];

        std::string dutyPath;

    public:
        Motor(uint8_t, int, bool, Settings&);
        void initPWM(void);
        void run(int);
    };


    class LeftMotor : public Motor {
    public:
        LeftMotor(Settings&);
    };

    class RightMotor : public Motor {
    public:
        RightMotor(Settings&);
    };




#endif //MOTORDAEMON_MOTOR_HPP
