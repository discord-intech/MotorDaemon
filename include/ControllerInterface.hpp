//
// Created by discord on 2/20/18.
//

#ifndef MOTORDAEMON_HERMES_CONTROLLERINTERFACE_HPP
#define MOTORDAEMON_HERMES_CONTROLLERINTERFACE_HPP

#include "Cinematic.hpp"

class ControllerInterface
{
public:

    ControllerInterface() = default;

    virtual void init(void) {};

    virtual void stop(void) {};

    virtual void orderTranslation(long) {};
    virtual void orderAngle(float) {};

    virtual void setSpeedTranslation(int) {};

    virtual  void orderCurveRadius(long) {};

    virtual  void setTranslationTunings(float, float, float) {};
    virtual  void setCurveTunings(float, float, float) {};
    virtual void setLeftSpeedTunings(float, float, float) {};
    virtual void setRightSpeedTunings(float, float, float) {};

    virtual void setPosition(double xn, double yn) {};
    virtual void setAngle(double o) {};

    virtual const char* getTunings(void) {return "";};

    virtual  void testPosition(void) {};

    virtual void testSpeed(int) {};

    virtual void setTrajectory(std::vector<Cinematic>&, long) {};

    virtual const char* isMoving(void) {return "";};

    virtual bool isPhysicallyStopped(void) {return 0;};

    virtual long getTranslationSetPoint(void) {return 0;};

    virtual void go(void) {};

    virtual void goR(void) {};

    virtual void setControlled(bool b) {};

    virtual const char* controlledStatus() { return "";};

    virtual void sweep(bool way) {};

    virtual void stopSweep(void) {};

    virtual long getCurveRadius(void) { return 0;};

    virtual double getX(void) {return 0;};

    virtual double getY(void) {return 0;};

    virtual long getSpeed(void) {return 0;};

    virtual long getSpeedL(void) {return 0;};

    virtual long getSpeedR(void) {return 0;};

    virtual long getCSpeedL(void) {return 0;};

    virtual long getCSpeedR(void) {return 0;};

    virtual double getAngle(void) {return 0;};

    virtual void loadPos() {};

    virtual void printTranslationError(void) {};
};

#endif //MOTORDAEMON_HERMES_CONTROLLERINTERFACE_HPP
