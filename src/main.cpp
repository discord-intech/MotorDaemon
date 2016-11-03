#include <iostream>
#include <vector>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdlib.h>
#include "../include/Odometry.hpp"
#include "../include/MotionController.hpp"

#define SOCKET_PORT 13337

void getArgs(const std::string&, char, std::vector<std::string>&);

void mainWorker();

int main(int argc, char *argv[])
{
    MotionController motion = MotionController();
    motion.init();

    char orderC[100];
    std::string order = "";
    std::vector<std::string> args = std::vector<std::string>();

    while(true)
    {
        std::cout << std::endl << "MotorDaemon Console : ";
        std::cin.getline(orderC, sizeof(orderC));
        order = std::string(orderC);
        if(order.length() == 0) continue;
        std::cout << std::endl;

        if(!order.compare("exit")) break;

        args.clear();
        getArgs(order, ' ', args);

        if(args.size() == 0) continue;

        if(!args[0].compare("stop"))
        {
            motion.stop();
            continue;
        }

        else if(!args[0].compare("setksg"))
        {

            if(args.size() != 4)
            {
                std::cout << "USAGE : setksg <kp> <ki> <kd>" << std::endl;
                continue;
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
                std::cout << "BAD VALUE" << std::endl;
                continue;
            }

            motion.setLeftSpeedTunings(kp, ki, kd);
            continue;
        }

        else if(!args[0].compare("setksd"))
        {

            if(args.size() != 4)
            {
                std::cout << "USAGE : setksd <kp> <ki> <kd>" << std::endl;
                continue;
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
                std::cout << "BAD VALUE" << std::endl;
                continue;
            }

            motion.setRightSpeedTunings(kp, ki, kd);
            continue;
        }

        else if(!args[0].compare("setktd"))
        {

            if(args.size() != 4)
            {
                std::cout << "USAGE : setktd <kp> <ki> <kd>" << std::endl;
                continue;
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
                std::cout << "BAD VALUE" << std::endl;
                continue;
            }

            motion.setTranslationTunings(kp, ki, kd);
            continue;
        }

        else if(!args[0].compare("sweep"))
        {
            Servo s = Servo(1100000, 0, 1550000, 180);

            s.initPWM();

            for(int i= 0; i < 180 ; i+=5)
            {
                std::cout << "angle : " << i << std::endl;
                s.setAngle(i);
                usleep(1000*1000);
            }

            continue;
        }

        else if(!args[0].compare("d"))
        {

            if(args.size() != 2)
            {
                std::cout << "USAGE : d <dist>" << std::endl;
                continue;
            }

            long dist;
            try
            {
                dist = std::stol(args[1]);
            }
            catch (std::exception const &e)
            {
                std::cout << "BAD VALUE" << std::endl;
                continue;
            }

            motion.orderTranslation(dist);
            continue;
        }

        else if(!args[0].compare("c"))
        {

            std::cout << motion.getOdometry()->getLeftValue() << " ; " << motion.getOdometry()->getRightValue() << std::endl;
            std::cout << motion.getCurveRadius() << std::endl << std::endl;
            continue;
        }

        else
        {
            std::cout << "No such command" << std::endl;
        }


    }

    return 0;
}

void getArgs(const std::string &s, char delim, std::vector<std::string> &elems)
{
    std::stringstream ss;
    ss.str(s);
    std::string item;

    while (getline(ss, item, delim)) {
        std::cout << item << std::endl;
        elems.push_back(item);
    }
}

void mainWorker(void)
{
    int sockfd; // socket file descriptor
    struct sockaddr_in serv_addr;
    struct hostent *server;

    sockfd = socket(AF_INET, SOCK_STREAM, 0); // generate file descriptor
    if (sockfd < 0)
    {
        perror("ERROR opening socket");
    }

    server = gethostbyname("127.0.0.1"); //the ip address (or server name) of the listening server.
    if (server == NULL)
    {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }

    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy(server->h_addr, (char *)&serv_addr.sin_addr.s_addr, (size_t) server->h_length);
    serv_addr.sin_port = htons(SOCKET_PORT);

    if (connect(sockfd,(struct sockaddr *)&serv_addr,sizeof(serv_addr)) < 0)
    {
        perror("ERROR connecting");
    }

    while(true)
    {
        char rbuff[256];
        ssize_t rbytes;

        //rbytes = read(sockfd, rbuff, sizeof(rbuff)); // read from socket and store the msg into buffer
        rbytes = recv(sockfd, rbuff, sizeof(rbuff), 0); // similar to read(), but return -1 if socket closed
        rbuff[rbytes] = '\0'; // set null terminal

        //TODO Comm inter-process
    }
}