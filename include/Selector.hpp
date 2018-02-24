//
// Created by discord on 16/11/16.
//

#ifndef MOTORDAEMON_SELECTOR_HPP
#define MOTORDAEMON_SELECTOR_HPP

#include "../include/Odometry.hpp"
#include "../include/MotionController.hpp"
#include "SerialController.hpp"
#include <sstream>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <sys/sysinfo.h>

volatile bool coprocessorMode = false;

#ifdef __arm__
Settings settings("/etc/MotorDaemon.conf");
#else
Settings settings("MotorDaemon.conf");
#endif

ControllerInterface* motion;



std::vector<std::string> args = std::vector<std::string>();

static bool serverMode = false;
static bool proxyMode = false;

std::vector<std::string> split(std::string str, char delimiter)
{
    std::vector<std::string> internal;
    std::stringstream ss(str); // Turn the string into a stream.
    std::string tok;

    while(std::getline(ss, tok, delimiter)) {
        internal.push_back(tok);
    }

    return internal;
}

unsigned long Millisec(void)
{
    struct timeval tv;
    if(gettimeofday(&tv, NULL) != 0) return 0;
    return (tv.tv_sec * 1000ul) + (tv.tv_usec / 1000ul);
}

void getArgs(const std::string &s, char delim, std::vector<std::string> &elems)
{
    std::stringstream ss(s);
    ss.str(s);
    std::string item;

    while (std::getline(ss, item, delim))
    {
        elems.push_back(item);
    }
}

double cpuUsage(void)
{
    long double a[4], b[4], loadavg;
    FILE *fp;
    char dump[50];

    fp = fopen("/proc/stat","r");
    fscanf(fp,"%*s %Lf %Lf %Lf %Lf",&a[0],&a[1],&a[2],&a[3]);
    fclose(fp);
    usleep(100000);

    fp = fopen("/proc/stat","r");
    fscanf(fp,"%*s %Lf %Lf %Lf %Lf",&b[0],&b[1],&b[2],&b[3]);
    fclose(fp);

    loadavg = ((b[0]+b[1]+b[2]) - (a[0]+a[1]+a[2])) / ((b[0]+b[1]+b[2]+b[3]) - (a[0]+a[1]+a[2]+a[3]));

    return loadavg;
}

double cpuTemp(void)
{
    if(settings.get("CPU_TEMP_FILE") == "null")
    {
        return 30.0;
    }

    FILE *fp;
    int t;

    fp = fopen(settings.get("CPU_TEMP_FILE").c_str(),"r");
    fscanf(fp, "%*s %d",t);

    return (double)t/1000.0;

}

std::string replaceAll(std::string str, const std::string& from, const std::string& to) {
    size_t start_pos = 0;
    while((start_pos = str.find(from, start_pos)) != std::string::npos) {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length(); // Handles case where 'to' is a substring of 'from'
    }
    return str;
}

int treatOrder(std::string &order, std::function<void(char*)> print, bool proxyMode = false)
{
    if(!order.compare("exit")) return 1;

    args.clear();
    getArgs(order, ' ', args);

    if(args.size() == 0) return 0;

    else if(!args[0].compare("startcamera"))
    {
        system(settings.get("CAMERA_GST_KILL").c_str());

        if(args.size() != 2 && !proxyMode)
        {
            print((char *) "USAGE : startcamera <IP_client>");
            return 0;
        }

        std::string s;
        if(!proxyMode || args.size()==2)
        {
            s = replaceAll(settings.get("CAMERA_GST_COMMAND"), std::string("%h"), std::string(args[1]));
        } else {
            s = replaceAll(settings.get("CAMERA_GST_COMMAND"), std::string("%h"), settings.get("IP_MOTORDAEMONPROXY"));
        }

        system(s.c_str());
    }

    else if(!args[0].compare("stopcamera"))
    {
        system(settings.get("CAMERA_GST_KILL").c_str());
    }

    else if(!args[0].compare("newmap"))
    {
        std::string json = "";

        for (long i=1; i<args.size() ; i++)
        {
            json.append(args[i]);
        }

        std::ofstream outfile (settings.get("MAP_FILE"),std::ofstream::binary);

        outfile.write(json.c_str(), json.length());
        outfile.flush();
        outfile.close();
    }

    else if(!args[0].compare("status"))
    {


        struct sysinfo sys;
        sysinfo(&sys);
        float cpuU = cpuUsage();
        float cpuT = cpuTemp();

        long speed = motion->getSpeed();



        std::string s = std::string("MDSTATUS")+std::to_string(cpuU)+std::string(";")+
                std::to_string(cpuT)+std::string(";")+std::to_string(sys.freeram)
                        +std::string(";")+std::to_string(sys.totalram)+std::string(";")+
                        std::to_string(speed)+std::string(";")+std::to_string(motion->getX())
                        +std::string(";")+std::to_string(motion->getY())+std::string(";")+
                        std::to_string(motion->getAngle())+std::string("");



        print((char *)s.c_str());


    }

    else if(!args[0].compare("stop"))
    {

        //print("ack");
        motion->stop();

        return 0;
    }
    else if(!args[0].compare("p"))
    {

        std::string s = std::to_string(motion->getX())+std::string(";")
                            +std::to_string(motion->getY())+std::string(";")
                            +std::to_string(motion->getAngle())+std::string("");
        print((char*)s.c_str());

        return 0;
    }
    else if(!args[0].compare("setksg"))
    {

        if(args.size() != 4)
        {
            print((char *) "USAGE : setksg <kp> <ki> <kd>");
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
            print((char *) "BAD VALUE");
            return 0;
        }

        motion->setLeftSpeedTunings(kp, ki, kd);

        return 0;
    }

    else if(!args[0].compare("setksd"))
    {

        if(args.size() != 4)
        {
            print((char *) "USAGE : setksd <kp> <ki> <kd>");
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
            print((char *) "BAD VALUE");
            return 0;
        }

        motion->setRightSpeedTunings(kp, ki, kd);

        return 0;
    }

    else if(!args[0].compare("setktd"))
    {

        if(args.size() != 4)
        {
            print((char *) "USAGE : setktd <kp> <ki> <kd>");
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
            print((char*)"BAD VALUE");
            return 0;
        }

        motion->setTranslationTunings(kp, ki, kd);

        return 0;
    }

    else if(!args[0].compare("setkcd"))
    {

        if(args.size() != 4)
        {
            print((char *) "USAGE : setkcd <kp> <ki> <kd>");
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
            print((char*)"BAD VALUE");
            return 0;
        }

        motion->setCurveTunings(kp, ki, kd);

        return 0;
    }

    else if(!args[0].compare("sweep"))
    {
        Servo s = Servo(1100000, 0, 1550000, 180);

        s.initPWM();

        for(int i= 0; i < 180 ; i+=5)
        {
            print((char*)("angle :" + std::to_string(i) + "").c_str());

            s.setAngle(i);
            usleep(1000*1000);
        }

        return 0;
    }

    else if(!args[0].compare("d"))
    {
        if(args.size() != 2)
        {
            print((char *) "USAGE : d <dist>");
            return 0;
        }

        long dist;
        try
        {
            dist = std::stol(args[1]);
        }
        catch (std::exception const &e)
        {
            print((char*)"BAD VALUE");
            return 0;
        }

        motion->orderTranslation(dist);

        return 0;
    }

    else if(!args[0].compare("followpath"))
    {
        if(args.size() != 2)
        {
            print((char *) "USAGE : followpath <dist1:curve1;dist2:curve2;...>");
            return 0;
        }


        std::vector<Cinematic> points = std::vector<Cinematic>();
        bool way;
        try
        {
            std::vector<std::string> strs = split(std::string(args[1]), ';');

            std::vector<std::string> su = split(strs[0], ':');
            way = su[1].compare("forward") == 0;

            for(std::string &s : strs)
            {
                std::vector<std::string> sub = split(s, ':');

                if(!sub[0].compare("way"))
                {
                    way = sub[1].compare("forward") == 0;
                    continue;
                }

                Cinematic c(std::stod(sub[0]), std::stod(sub[1]), way);
                points.push_back(c);
            }
        }
        catch (std::exception const &e)
        {
            print((char*)"BAD VALUE");
            return 0;
        }


        motion->setTrajectory(points, points[points.size()-1].relativeDistance);

        return 0;
    }

    else if(!args[0].compare("go"))
    {


        motion->setControlled(false);
        motion->go();

        std::cout << "go received" << std::endl;
        return 0;
    }

    else if(!args[0].compare("setpos"))
    {
        if(args.size() != 3)
        {
            print((char *) "USAGE : setpos <x> <y>");
            return 0;
        }

        double x, y;
        try
        {
            x = std::stod(args[1]);
            y = std::stod(args[2]);
        }
        catch (std::exception const &e)
        {
            print((char*)"BAD VALUE");
            return 0;
        }

        motion->setPosition(x,y);

        return 0;
    }

    else if(!args[0].compare("setang"))
    {
        if(args.size() != 2)
        {
            print((char *) "USAGE : setang <rad>");
            return 0;
        }

        double o;
        try
        {
            o = std::stod(args[1]);
        }
        catch (std::exception const &e)
        {
            print((char*)"BAD VALUE");
            return 0;
        }

        motion->setAngle(o);

        return 0;
    }

    else if(!args[0].compare("gor"))
    {


        motion->setControlled(false);
        motion->goR();

        std::cout << "gor received" << std::endl;
        return 0;
    }

    else if(!args[0].compare("sweepR"))
    {


        motion->sweep(true);

        std::cout << "sweepR received" << std::endl;
        return 0;
    }

    else if(!args[0].compare("sweepL"))
    {


        motion->sweep(false);

        std::cout << "sweepL received" << std::endl;
        return 0;
    }

    else if(!args[0].compare("sweepstop"))
    {


        motion->stopSweep();

        return 0;
    }

    else if(!args[0].compare("cr"))
    {
        if(args.size() != 2)
        {
            print((char *) "USAGE : cr <rad>");
            return 0;
        }

        long dist;
        try
        {
            dist = std::stol(args[1]);
        }
        catch (std::exception const &e)
        {
            print((char*)"BAD VALUE");
            return 0;
        }

        motion->orderCurveRadius(dist);

        return 0;
    }

    else if(!args[0].compare("seta"))
    {
        if(args.size() != 2)
        {
            print((char *) "USAGE : seta <angle in radians>");
            return 0;
        }

        float angle;
        try
        {
            angle = std::stof(args[1]);
        }
        catch (std::exception const &e)
        {
            print((char*)"BAD VALUE");
            return 0;
        }

        motion->orderAngle(angle);

        return 0;
    }

    else if(!args[0].compare("sets"))
    {
        if(args.size() != 2)
        {
            print((char *) "USAGE : sets <speed in ticks/s>");
            return 0;
        }

        int s;
        try
        {
            s = std::stoi(args[1]);
        }
        catch (std::exception const &e)
        {
            print((char*)"BAD VALUE");
            return 0;
        }

        motion->setSpeedTranslation(s);

        return 0;
    }

    else if(!args[0].compare("ts"))
    {

        if(args.size() != 2)
        {
            print((char *) "USAGE : ts <speed>");
            return 0;
        }

        long dist;
        try
        {
            dist = std::stol(args[1]);
        }
        catch (std::exception const &e)
        {
            print((char*)"BAD VALUE");
            return 0;
        }


        motion->testSpeed(dist);


        return 0;
    }

    else if(!args[0].compare("c"))
    {

        print((char*)(
//                std::to_string(motion->getOdometry()->getLeftValue())+std::string(" ; ")+std::to_string(motion->getOdometry()->getRightValue())+std::string("\n")+
                std::to_string(motion->getCurveRadius())+std::string("\n")).c_str());

        return 0;
    }

    else if(!args[0].compare("k"))
    {

        print((char*) motion->getTunings());

        return 0;
    }

    else if(!args[0].compare("m"))
    {

        print((char*) motion->isMoving());

        return 0;
    }

    else if(!args[0].compare("sv"))
    {

        std::string s = std::to_string(Millisec())+std::string(";")+std::to_string(motion->getCSpeedL())+std::string(";")+std::to_string(motion->getSpeedL())+
            std::string(";")+std::to_string(motion->getCSpeedR())+std::string(";")+std::to_string(motion->getSpeedR())+std::string("");
        print((char*)s.c_str());

        return 0;
    }

    else if(!args[0].compare("isOn"))
    {
        print((char*)motion->controlledStatus());
        return 0;
    }

    else
    {
        print((char *) "No such command");
    }

    return 0;

}

#endif //MOTORDAEMON_SELECTOR_HPP
