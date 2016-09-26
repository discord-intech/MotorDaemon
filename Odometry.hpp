//
// Created by discord on 26/09/16.
//

#ifndef MOTORDAEMON_ODOMETRY_HPP
#define MOTORDAEMON_ODOMETRY_HPP


#include <cstdint>
#include <glib.h>

class Odometry {


private:

    static long leftTicks = 0;
    static long rightTicks = 0;

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

#endif //MOTORDAEMON_ODOMETRY_HPP
