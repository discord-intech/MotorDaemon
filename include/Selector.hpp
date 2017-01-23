//
// Created by discord on 16/11/16.
//

#ifndef MOTORDAEMON_SELECTOR_HPP
#define MOTORDAEMON_SELECTOR_HPP


#include "../include/Odometry.hpp"
#include "../include/MotionController.hpp"
#include <sstream>

#ifdef __arm__
MotionController motion;
#endif
std::vector<std::string> args = std::vector<std::string>();

static bool serverMode = false;



void getArgs(const std::string &s, char delim, std::vector<std::string> &elems)
{
    std::stringstream ss(s);
    ss.str(s);
    std::string item;

    while (getline(ss, item, delim))
    {
        elems.push_back(item);
    }
}


int treatOrder(std::string &order, std::function<void(char*)> print)
{
    if(!order.compare("exit")) return 1;

    args.clear();
    getArgs(order, ' ', args);

    if(args.size() == 0) return 0;

    if(!args[0].compare("stop"))
    {
#ifdef __arm__
        print("Ordering to stop\r\n");
        motion.stop();
#endif
        return 0;
    }

    else if(!args[0].compare("setksg"))
    {

        if(args.size() != 4)
        {
            print((char *) "USAGE : setksg <kp> <ki> <kd>\r\n");
            return 0;
        }

        float kp, ki, kd;
        try
        {
            kp = std::stof(args[1]);
            ki = std::stof(args[2]);
            kd = std::stof(args[3]);
        }
        catch (std::exception const &e)
        {
            print((char *) "BAD VALUE\r\n");
            return 0;
        }
#ifdef __arm__
        motion.setLeftSpeedTunings(kp, ki, kd);
#endif
        return 0;
    }

    else if(!args[0].compare("setksd"))
    {

        if(args.size() != 4)
        {
            print((char *) "USAGE : setksd <kp> <ki> <kd>\r\n");
            return 0;
        }

        float kp, ki, kd;
        try
        {
            kp = std::stof(args[1]);
            ki = std::stof(args[2]);
            kd = std::stof(args[3]);
        }
        catch (std::exception const &e)
        {
            print((char *) "BAD VALUE\r\n");
            return 0;
        }
#ifdef __arm__
        motion.setRightSpeedTunings(kp, ki, kd);
#endif
        return 0;
    }

    else if(!args[0].compare("setktd"))
    {

        if(args.size() != 4)
        {
            print((char *) "USAGE : setktd <kp> <ki> <kd>\r\n");
            return 0;
        }

        float kp, ki, kd;
        try
        {
            kp = std::stof(args[1]);
            ki = std::stof(args[2]);
            kd = std::stof(args[3]);
        }
        catch (std::exception const &e)
        {
            print((char*)"BAD VALUE\r\n");
            return 0;
        }
#ifdef __arm__
        motion.setTranslationTunings(kp, ki, kd);
#endif
        return 0;
    }

    else if(!args[0].compare("sweep"))
    {
        Servo s = Servo(1100000, 0, 1550000, 180);

        s.initPWM();

        for(int i= 0; i < 180 ; i+=5)
        {
            print((char*)("angle :" + std::to_string(i) + "\r\n").c_str());

            s.setAngle(i);
            usleep(1000*1000);
        }

        return 0;
    }

    else if(!args[0].compare("d"))
    {
        if(args.size() != 2)
        {
            print((char *) "USAGE : d <dist>\r\n");
            return 0;
        }

        long dist;
        try
        {
            dist = std::stol(args[1]);
        }
        catch (std::exception const &e)
        {
            print((char*)"BAD VALUE\r\n");
            return 0;
        }
#ifdef __arm__
        motion.orderTranslation(dist);
#endif
        return 0;
    }

    else if(!args[0].compare("go"))
    {

#ifdef __arm__
        motion.setControlled(false);
        motion.go();
#endif
        return 0;
    }

    else if(!args[0].compare("sweepR"))
    {

#ifdef __arm__
        motion.sweep(false);
#endif
        return 0;
    }

    else if(!args[0].compare("sweepL"))
    {

#ifdef __arm__
        motion.sweep(true);
#endif
        return 0;
    }

    else if(!args[0].compare("sweepstop"))
    {

#ifdef __arm__
        motion.stopSweep();
#endif
        return 0;
    }

    else if(!args[0].compare("cr"))
    {
        if(args.size() != 2)
        {
            print((char *) "USAGE : cr <rad>\r\n");
            return 0;
        }

        long dist;
        try
        {
            dist = std::stol(args[1]);
        }
        catch (std::exception const &e)
        {
            print((char*)"BAD VALUE\r\n");
            return 0;
        }
#ifdef __arm__
        motion.orderCurveRadius(dist);
#endif
        return 0;
    }

    else if(!args[0].compare("seta"))
    {
        if(args.size() != 2)
        {
            print((char *) "USAGE : seta <angle in radians>\r\n");
            return 0;
        }

        float angle;
        try
        {
            angle = std::stof(args[1]);
        }
        catch (std::exception const &e)
        {
            print((char*)"BAD VALUE\r\n");
            return 0;
        }
#ifdef __arm__
        motion.orderAngle(angle);
#endif
        return 0;
    }

    else if(!args[0].compare("sets"))
    {
        if(args.size() != 2)
        {
            print((char *) "USAGE : sets <speed in ticks/s>\r\n");
            return 0;
        }

        int s;
        try
        {
            s = std::stoi(args[1]);
        }
        catch (std::exception const &e)
        {
            print((char*)"BAD VALUE\r\n");
            return 0;
        }
#ifdef __arm__
        motion.setSpeedTranslation(s);
#endif
        return 0;
    }

    else if(!args[0].compare("ts"))
    {

        if(args.size() != 2)
        {
            print((char *) "USAGE : ts <speed>\r\n");
            return 0;
        }

        long dist;
        try
        {
            dist = std::stol(args[1]);
        }
        catch (std::exception const &e)
        {
            print((char*)"BAD VALUE\r\n");
            return 0;
        }

        print((char*)"Speed test launched !\r\n");

#ifdef __arm__
        motion.testSpeed(dist);
#endif
        print((char*)"Speed test ended !\r\n");

        return 0;
    }

    else if(!args[0].compare("c"))
    {
#ifdef __arm__
        print((char*)(
                std::to_string(motion.getOdometry()->getLeftValue())+std::string(" ; ")+std::to_string(motion.getOdometry()->getRightValue())+std::string("\r\n")+
                std::to_string(motion.getCurveRadius())+std::string("\r\n\n")).c_str());
#endif
        return 0;
    }

    else if(!args[0].compare("k"))
    {
#ifdef __arm__
        print((char*) motion.getTunings());
#endif
        return 0;
    }

    else
    {
        print((char *) "No such command\r\n");
    }

    return 0;

}

#endif //MOTORDAEMON_SELECTOR_HPP
