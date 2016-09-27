//
// Created by discord on 26/09/16.
//

#include <fcntl.h>
#include <string>
#include "Odometry.hpp"


Odometry::Odometry(uint8_t chanAL, uint8_t chanBL, uint8_t chanAR, uint8_t chanBR)
{
    //TODO change pins
    int fdAL = open( (std::string("/sys/class/gpio/gpio")+std::to_string(chanAL)+std::string("/value")).c_str(), O_RDONLY | O_NONBLOCK );
    GIOChannel* channelAL = g_io_channel_unix_new( fdAL );
    GIOCondition cond = GIOCondition( G_IO_PRI );
    guint idAL = g_io_add_watch(channelAL, cond, onTickChanALeft, 0);

    int fdBL = open( (std::string("/sys/class/gpio/gpio")+std::to_string(chanBL)+std::string("/value")).c_str(), O_RDONLY | O_NONBLOCK );
    GIOChannel* channelBL = g_io_channel_unix_new( fdBL );
    guint idBL = g_io_add_watch(channelBL, cond, onTickChanBLeft, 0);

    int fdAR = open( (std::string("/sys/class/gpio/gpio")+std::to_string(chanAR)+std::string("/value")).c_str(), O_RDONLY | O_NONBLOCK );
    GIOChannel* channelAR = g_io_channel_unix_new( fdAR );
    guint idAR = g_io_add_watch(channelAR, cond, onTickChanARight, 0);

    int fdBR = open( (std::string("/sys/class/gpio/gpio")+std::to_string(chanBR)+std::string("/value")).c_str(), O_RDONLY | O_NONBLOCK );
    GIOChannel* channelBR = g_io_channel_unix_new( fdBR );
    guint idBR = g_io_add_watch(channelBR, cond, onTickChanBRight, 0);

    Odometry::leftTicks = 0;
    Odometry::rightTicks = 0;
    Odometry::firstChanL = 0;
    Odometry::firstChanR = 0;

}

long Odometry::getLeftValue() {
    return Odometry::leftTicks;
}

long Odometry::getRightValue() {
    return Odometry::rightTicks;
}

gboolean Odometry::onTickChanALeft(GIOChannel *channel,
                                   GIOCondition condition,
                                   gpointer user_data)
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

    GError *error = 0;
    char buf;
    unsigned long buf_sz = 1;
    unsigned long bytes_read = 0;

    g_io_channel_seek_position( channel, 0, G_SEEK_SET, 0 );
    GIOStatus rc = g_io_channel_read_chars( channel,
                                            &buf, buf_sz,
                                            &bytes_read,
                                            &error );


    return 1;
}

gboolean Odometry::onTickChanBLeft(GIOChannel *channel,
                                   GIOCondition condition,
                                   gpointer user_data)
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

    GError *error = 0;
    char buf;
    unsigned long buf_sz = 1;
    unsigned long bytes_read = 0;

    g_io_channel_seek_position( channel, 0, G_SEEK_SET, 0 );
    GIOStatus rc = g_io_channel_read_chars( channel,
                                            &buf, buf_sz,
                                            &bytes_read,
                                            &error );


    return 1;
}

gboolean Odometry::onTickChanARight(GIOChannel *channel,
                                   GIOCondition condition,
                                   gpointer user_data)
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

    GError *error = 0;
    char buf;
    unsigned long buf_sz = 1;
    unsigned long bytes_read = 0;

    g_io_channel_seek_position( channel, 0, G_SEEK_SET, 0 );
    GIOStatus rc = g_io_channel_read_chars( channel,
                                            &buf, buf_sz,
                                            &bytes_read,
                                            &error );

    return 1;
}

gboolean Odometry::onTickChanBRight(GIOChannel *channel,
                                   GIOCondition condition,
                                   gpointer user_data)
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

    GError *error = 0;
    char buf;
    unsigned long buf_sz = 1;
    unsigned long bytes_read = 0;

    g_io_channel_seek_position( channel, 0, G_SEEK_SET, 0 );
    GIOStatus rc = g_io_channel_read_chars( channel,
                                            &buf, buf_sz,
                                            &bytes_read,
                                            &error );

    return 1;
}
