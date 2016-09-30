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

#define PWM_TIME_PERIOD 100000  // nanosecondes


    struct direction_def {
        enum type {
            BACKWARD, FORWARD
        };
    };

    struct side_def {
        enum type {
            LEFT, RIGHT
        };
    };

    typedef safe_enum<direction_def> Direction;
    typedef safe_enum<side_def> Side;

    class Motor {
    private:
        Side side;
        BlackLib::BlackPWM pwm = BlackLib::BlackPWM(BlackLib::PWMDISABLE);
        BlackLib::pwmName PWMpin;

        BlackLib::BlackGPIO directionPin1 = BlackLib::BlackGPIO(BlackLib::GPIO_2, BlackLib::input); //placeholder
        BlackLib::BlackGPIO directionPin2 = BlackLib::BlackGPIO(BlackLib::GPIO_2, BlackLib::input); //placeholder

        Direction actualDirection = Direction::BACKWARD; //Changed to FORWARD in init
        void setDirection(Direction);

    public:
        Motor(Side);
        void initPWM(void);
        void run(long);
    };


#endif //MOTORDAEMON_MOTOR_HPP
