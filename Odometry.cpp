//
// Created by discord on 26/09/16.
//

#include <fcntl.h>
#include "Odometry.hpp"


Odometry::Odometry()
{
    //TODO change pins
    int fdL = open( "/sys/class/gpio/gpio7/value", O_RDONLY | O_NONBLOCK );
    GIOChannel* channelL = g_io_channel_unix_new( fdL );
    GIOCondition cond = GIOCondition( G_IO_PRI );
    guint idL = g_io_add_watch( channelL, cond, onTickLeft, 0 );

    int fdR = open( "/sys/class/gpio/gpio7/value", O_RDONLY | O_NONBLOCK );
    GIOChannel* channelR = g_io_channel_unix_new( fdL );
    guint idR = g_io_add_watch( channelL, cond, onTickRight, 0 );

    Odometry::leftTicks = 0;
    Odometry::rightTicks = 0;

}

long Odometry::getLeftValue() {
    return Odometry::leftTicks;
}

long Odometry::getRightValue() {
    return Odometry::rightTicks;
}

gboolean Odometry::onTickLeft(GIOChannel *channel,
                              GIOCondition condition,
                              gpointer user_data)
{
    GError *error = 0;
    char buf;
    unsigned long buf_sz = 1;
    unsigned long bytes_read = 0;

    g_io_channel_seek_position( channel, 0, G_SEEK_SET, 0 );
    GIOStatus rc = g_io_channel_read_chars( channel,
                                            &buf, buf_sz,
                                            &bytes_read,
                                            &error );

    //TODO counter + détection de sens

    return 1;
}

gboolean Odometry::onTickRight(GIOChannel *channel,
                               GIOCondition condition,
                               gpointer user_data)
{
    GError *error = 0;
    char buf;
    unsigned long buf_sz = 1;
    unsigned long bytes_read = 0;

    g_io_channel_seek_position( channel, 0, G_SEEK_SET, 0 );
    GIOStatus rc = g_io_channel_read_chars( channel,
                                            &buf, buf_sz,
                                            &bytes_read,
                                            &error );

    //TODO counter + détection de sens

    return 1;
}
