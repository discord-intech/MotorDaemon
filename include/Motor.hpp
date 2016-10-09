//
// Created by discord on 26/09/16.
//

#ifndef MOTORDAEMON_MOTOR_HPP
#define MOTORDAEMON_MOTOR_HPP

#include <cstdint>
#include "safe_enum.hpp"
#include <BlackPWM.h>
#include <BlackGPIO.h>

#define MIN(x,y) (((x)<(y))?(x):(y))
#define MAX(x,y) (((x)>(y))?(x):(y))
#define ABS(x) (((x) > 0) ? (x) : -(x))
#define SIN(x) (1-(((x)*(x)*(x))/6))
#define COS(x) (1-(((x)*(x))/4))
#define ARCTAN(x) (1-(((x)*(x)*(x))/3))
#define TAN(x) ((x)+(((x)*(x)*(x))/3))

#define PWM_TIME_PERIOD 1000*1000  // nanosecondes


    struct direction_def {
        enum type {
            BACKWARD, FORWARD
        };
    };

    typedef safe_enum<direction_def> Direction;

    class Motor {
    private:
        BlackLib::BlackPWM pwm = BlackLib::BlackPWM(BlackLib::PWMDISABLE);
        BlackLib::pwmName PWMpin;

        BlackLib::BlackGPIO directionPin1; //placeholder
        BlackLib::BlackGPIO directionPin2; //placeholder

        Direction actualDirection = Direction::BACKWARD; //Changed to FORWARD in init
        void setDirection(Direction);
        void setDirectionPins(void);

    public:
        Motor(BlackLib::pwmName, BlackLib::BlackGPIO, BlackLib::BlackGPIO);
        void initPWM(void);
        void run(long);
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
