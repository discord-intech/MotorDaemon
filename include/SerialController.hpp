//
// Created by discord on 2/20/18.
//

#ifndef MOTORDAEMON_HERMES_SERIALCONTROLLER_HPP
#define MOTORDAEMON_HERMES_SERIALCONTROLLER_HPP

#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <cstdlib>
#include <sstream>
#include <fstream>
#include <functional>
#include <thread>
#include <unistd.h>
#include <queue>
#include <iostream>
#include "cpu_com_structs.h"
#include "ControllerInterface.hpp"

typedef struct cpu_com_result Result;

#define SERIAL_BAUDRATE 115200

class SerialController : public ControllerInterface
{
private:
    static int fileDesc;
    static struct cpu_com_status* currentStatus;

    std::thread main;
    std::thread reader;

    static std::queue<Result*> resultQueue;

    static int Read(char*, int);

    static void mainWorker(void);
    static void readWorker(void);

public:
    static bool on;
    SerialController(char*);
    void init();
    int Write(const char*, unsigned int);
    Result* waitForResult();
    struct cpu_com_status getStatus();
    void order(std::string order);

    void stop(void) ;

    void orderTranslation(long i) ;

    void orderAngle(float d) ;

    void setSpeedTranslation(int i) ;

    void orderCurveRadius(long i) ;

    void setTranslationTunings(float d, float d1, float d2) ;

    void setCurveTunings(float d, float d1, float d2) ;

    void setLeftSpeedTunings(float d, float d1, float d2) ;

    void setRightSpeedTunings(float d, float d1, float d2) ;

    void setPosition(double xn, double yn) ;

    void setAngle(double o) ;

    const char *getTunings(void) ;

    void testPosition(void) ;

    void testSpeed(int i) ;

    void setTrajectory(std::vector<Cinematic> &vector, long i) ;

    const char *isMoving(void) ;

    bool isPhysicallyStopped(void) ;

    long getTranslationSetPoint(void) ;

    void go(void) ;

    void goR(void) ;

    void setControlled(bool b) ;

    void sweep(bool way) ;

    void stopSweep(void) ;

    long getCurveRadius(void) ;

    double getX(void) ;

    double getY(void) ;

    long getSpeed(void) ;

    long getSpeedL(void) ;

    long getSpeedR(void) ;

    long getCSpeedL(void) ;

    long getCSpeedR(void) ;

    double getAngle(void) ;

    void loadPos() ;

    void printTranslationError(void) ;

    const char *controlledStatus() ;

    std::vector<std::string> splitl(std::string str, char delimiter)
    {
        std::vector<std::string> internal;
        std::stringstream ss(str); // Turn the string into a stream.
        std::string tok;

        while(std::getline(ss, tok, delimiter)) {
            internal.push_back(tok);
        }

        return internal;
    }

    int set_interface_attribs (int fd, int speed, int parity)
    {
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
            printf ("error from tcgetattr", errno);
            return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // ignore break signal
        tty.c_lflag = 0;                // no signaling chars, no echo,
        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
            printf ("error %d from tcsetattr", errno);
            return -1;
        }
        return 0;
    }

    void set_blocking (int fd, int should_block)
    {
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
            printf ("error %d from tggetattr", errno);
            return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            	// 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
            printf ("error %d setting term attributes", errno);
    }
};




#endif //MOTORDAEMON_HERMES_SERIALCONTROLLER_HPP
