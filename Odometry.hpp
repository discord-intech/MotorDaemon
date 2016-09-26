//
// Created by discord on 26/09/16.
//

#ifndef MOTORDAEMON_ODOMETRY_HPP
#define MOTORDAEMON_ODOMETRY_HPP


#include <cstdint>
#include <glib.h>

class Odometry {


private:

    static long leftTicks;
    static long rightTicks;

    static gboolean onTickLeft(GIOChannel *channel,
                               GIOCondition condition,
                               gpointer user_data);
    static gboolean onTickRight(GIOChannel *channel,
                                GIOCondition condition,
                                gpointer user_data);

public:
    Odometry();
    long getLeftValue();
    long getRightValue();
};

long Odometry::leftTicks;
long Odometry::rightTicks;

#endif //MOTORDAEMON_ODOMETRY_HPP
