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

    virtual void init(void) = 0;

    virtual void destructor(void) = 0;

    virtual void stop(void) = 0;

    virtual void orderTranslation(long) = 0;
    virtual void orderAngle(float) = 0;

    virtual void setSpeedTranslation(int) = 0;

    virtual  void orderCurveRadius(long) = 0;

    virtual  void setTranslationTunings(float, float, float) = 0;
    virtual  void setCurveTunings(float, float, float) = 0;
    virtual void setLeftSpeedTunings(float, float, float) = 0;
    virtual void setRightSpeedTunings(float, float, float) = 0;

    virtual void setPosition(double xn, double yn) = 0;
    virtual void setAngle(double o) = 0;

    virtual const char* getTunings(void) = 0;

    virtual  void testPosition(void) = 0;

    virtual void testSpeed(int) = 0;

    virtual void setTrajectory(std::vector<Cinematic>&, long) = 0;

    virtual const char* isMoving(void) = 0;

    virtual bool isPhysicallyStopped(void) = 0;

    virtual long getTranslationSetPoint(void) = 0;

    virtual void go(void) = 0;

    virtual void goR(void) = 0;

    virtual void setControlled(bool b) = 0;

    virtual const char* controlledStatus() = 0;

    virtual void sweep(bool way) = 0;

    virtual void stopSweep(void) = 0;

    virtual long getCurveRadius(void) = 0;

    virtual double getX(void) = 0;

    virtual double getY(void) = 0;

    virtual long getSpeed(void) = 0;

    virtual long getSpeedL(void) = 0;

    virtual long getSpeedR(void) = 0;

    virtual long getCSpeedL(void) = 0;

    virtual long getCSpeedR(void) = 0;

    virtual double getAngle(void) = 0;

    virtual void loadPos() = 0;

    virtual void printTranslationError(void) = 0;

    virtual void setNeonSpeed(unsigned char s) = 0;
};

#endif //MOTORDAEMON_HERMES_CONTROLLERINTERFACE_HPP
