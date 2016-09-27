//
// Created by discord on 26/09/16.
//

#include <fcntl.h>
#include <poll.h>
#include <string>
#include <stdlib.h>
#include "Odometry.hpp"

#define AL 0
#define BL 1
#define AR 2
#define BR 3

long Odometry::leftTicks;
long Odometry::rightTicks;
uint8_t Odometry::firstChanL; //1 : chanA ; 2 : chanB
uint8_t Odometry::firstChanR; //1 : chanA ; 2 : chanB

Odometry::Odometry(uint8_t chanAL, uint8_t chanBL, uint8_t chanAR, uint8_t chanBR)
{
    system((std::string("echo ")+std::to_string(chanAL)+std::string(" > /sys/class/gpio/export")).c_str());
    system((std::string("echo ")+std::to_string(chanBL)+std::string(" > /sys/class/gpio/export")).c_str());
    system((std::string("echo ")+std::to_string(chanAR)+std::string(" > /sys/class/gpio/export")).c_str());
    system((std::string("echo ")+std::to_string(chanBR)+std::string(" > /sys/class/gpio/export")).c_str());

    system((std::string("echo in > /sys/class/gpio/gpio")+std::to_string(chanAL)+std::string("/direction")).c_str());
    system((std::string("echo in > /sys/class/gpio/gpio")+std::to_string(chanBL)+std::string("/direction")).c_str());
    system((std::string("echo in > /sys/class/gpio/gpio")+std::to_string(chanAR)+std::string("/direction")).c_str());
    system((std::string("echo in > /sys/class/gpio/gpio")+std::to_string(chanBR)+std::string("/direction")).c_str());

    system((std::string("echo rising > /sys/class/gpio/gpio")+std::to_string(chanAL)+std::string("/edge")).c_str());
    system((std::string("echo rising > /sys/class/gpio/gpio")+std::to_string(chanBL)+std::string("/edge")).c_str());
    system((std::string("echo rising > /sys/class/gpio/gpio")+std::to_string(chanAR)+std::string("/edge")).c_str());
    system((std::string("echo rising > /sys/class/gpio/gpio")+std::to_string(chanBR)+std::string("/edge")).c_str());

    //TODO change pins
    int fdAL = open( (std::string("/sys/class/gpio/gpio")+std::to_string(chanAL)+std::string("/value")).c_str(), O_RDONLY | O_NONBLOCK );
    int fdBL = open( (std::string("/sys/class/gpio/gpio")+std::to_string(chanBL)+std::string("/value")).c_str(), O_RDONLY | O_NONBLOCK );
    int fdAR = open( (std::string("/sys/class/gpio/gpio")+std::to_string(chanAR)+std::string("/value")).c_str(), O_RDONLY | O_NONBLOCK );
    int fdBR = open( (std::string("/sys/class/gpio/gpio")+std::to_string(chanBR)+std::string("/value")).c_str(), O_RDONLY | O_NONBLOCK );

    Odometry::leftTicks = 0;
    Odometry::rightTicks = 0;
    Odometry::firstChanL = 0;
    Odometry::firstChanR = 0;

    t = std::thread(std::bind(mainWorker, chanAL, chanBL, chanAR, chanBR), "Counter Thread");
    t.detach();
}

void Odometry::mainWorker(uint8_t chanAL, uint8_t chanBL, uint8_t chanAR, uint8_t chanBR)
{
    int fdAL = open( (std::string("/sys/class/gpio/gpio")+std::to_string(chanAL)+std::string("/value")).c_str(), O_RDONLY | O_NONBLOCK );
    int fdBL = open( (std::string("/sys/class/gpio/gpio")+std::to_string(chanBL)+std::string("/value")).c_str(), O_RDONLY | O_NONBLOCK );
    int fdAR = open( (std::string("/sys/class/gpio/gpio")+std::to_string(chanAR)+std::string("/value")).c_str(), O_RDONLY | O_NONBLOCK );
    int fdBR = open( (std::string("/sys/class/gpio/gpio")+std::to_string(chanBR)+std::string("/value")).c_str(), O_RDONLY | O_NONBLOCK );

    struct pollfd pfd[4];

    pfd[AL].fd = fdAL;
    pfd[AL].events = POLLPRI;
    pfd[AL].revents = 0;

    pfd[BL].fd = fdBL;
    pfd[BL].events = POLLPRI;
    pfd[BL].revents = 0;

    pfd[AR].fd = fdAR;
    pfd[AR].events = POLLPRI;
    pfd[AR].revents = 0;

    pfd[BR].fd = fdBR;
    pfd[BR].events = POLLPRI;
    pfd[BR].revents = 0;

    while (true)
    {
        poll(pfd, 4, -1);

        if (pfd[AL].revents != 0) {
            onTickChanALeft();
        }
        if (pfd[BL].revents != 0) {
            onTickChanBLeft();
        }
        if (pfd[AR].revents != 0) {
            onTickChanARight();
        }
        if (pfd[BR].revents != 0) {
            onTickChanBRight();
        }

        usleep(2);
    }
}

long Odometry::getLeftValue() {
    return Odometry::leftTicks;
}

long Odometry::getRightValue() {
    return Odometry::rightTicks;
}

void Odometry::onTickChanALeft(void)
{
    if(firstChanL == 0)
    {
        firstChanL = 1;
    }
    if(firstChanL == 2)
    {
        firstChanL = 0;
        leftTicks--;
    }
}

void Odometry::onTickChanBLeft(void)
{
    if(firstChanL == 0)
    {
        firstChanL = 2;
    }
    if(firstChanL == 1)
    {
        firstChanL = 0;
        leftTicks++;
    }
}

void Odometry::onTickChanARight(void)
{
    if(firstChanR == 0)
    {
        firstChanR = 1;
    }
    if(firstChanR == 2)
    {
        firstChanR = 0;
        rightTicks--;
    }
}

void Odometry::onTickChanBRight(void)
{
    if(firstChanR == 0)
    {
        firstChanR = 2;
    }
    if(firstChanR == 1)
    {
        firstChanR = 0;
        rightTicks++;
    }
}

