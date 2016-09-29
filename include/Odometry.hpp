//
// Created by discord on 26/09/16.
//

#ifndef MOTORDAEMON_ODOMETRY_HPP
#define MOTORDAEMON_ODOMETRY_HPP


#include <cstdint>
#include <unistd.h>
#include <thread>



class Odometry {


private:

    std::thread t;
    static long leftTicks;
    static long rightTicks;
    static int valueAL;
    static int valueBL;
    static int valueAR;
    static int valueBR;


    static void mainWorker(uint8_t chanAL, uint8_t chanBL, uint8_t chanAR, uint8_t chanBR);

    static void onTickChanALeft(void);
    static void onTickChanBLeft(void);
    static void onTickChanARight(void);
    static void onTickChanBRight(void);
    static void get_lead(int&, uint8_t);

public:
    Odometry(uint8_t chanAL, uint8_t chanBL, uint8_t chanAR, uint8_t chanBR);
    long getLeftValue();
    long getRightValue();

};



#endif //MOTORDAEMON_ODOMETRY_HPP
