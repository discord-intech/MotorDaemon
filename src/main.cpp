#include <iostream>
#include <vector>
#include <string>
#include <sys/types.h>
#include <sys/un.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <syslog.h>
#include <netdb.h>
#include <stdlib.h>
#include "../include/Odometry.hpp"
#include "../include/MotionController.hpp"

#define SOCKET_PORT 56987
#define SERVER_MODE_CMD "-s"
#define DAEMON_NAME "motordaemon"

void getArgs(const std::string&, char, std::vector<std::string>&);
int writeMessage(int &sockfd, std::string &str);
void serverWorker(void);
void localWorker(void);
int treatOrder(std::string &order);

#ifdef __arm__
MotionController motion;
#endif
std::vector<std::string> args = std::vector<std::string>();


int main(int argc, char *argv[])
{
    setlogmask(LOG_UPTO(LOG_NOTICE));
    openlog(DAEMON_NAME, LOG_CONS | LOG_NDELAY | LOG_PERROR | LOG_PID, LOG_USER);

#ifdef __arm__
    motion = MotionController();
    motion.init();
#endif

    if(argc >= 2 && !strcmp(argv[1], SERVER_MODE_CMD))
    {
        std::thread t(serverWorker);
        syslog(LOG_INFO, "MotorDaemon launched in server mode");
        t.join(); //Do not shut down the main thread
    }
    else
    {
        std::thread t(localWorker);
        syslog(LOG_INFO, "MotorDaemon launched in user mode");
        t.join(); //Do not shut down the main thread
    }

}

void localWorker(void)
{

    char orderC[100];
    std::string order = "";

    while(true)
    {
        std::cout << std::endl << "MotorDaemon Console : ";
        std::cin.getline(orderC, sizeof(orderC));
        order = std::string(orderC);
        if(order.length() == 0) continue;
        std::cout << std::endl;

        if(treatOrder(order))
        {
#ifdef __arm__
            motion.stop();
#endif
            exit(0);
        }
    }

}

int treatOrder(std::string &order)
{
    if(!order.compare("exit")) return 1;

    args.clear();
    getArgs(order, ' ', args);

    if(args.size() == 0) return 0;

    if(!args[0].compare("stop"))
    {
#ifdef __arm__
        motion.stop();
#endif
        return 0;
    }

    else if(!args[0].compare("setksg"))
    {

        if(args.size() != 4)
        {
            std::cout << "USAGE : setksg <kp> <ki> <kd>" << std::endl;
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
            std::cout << "BAD VALUE" << std::endl;
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
            std::cout << "USAGE : setksd <kp> <ki> <kd>" << std::endl;
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
            std::cout << "BAD VALUE" << std::endl;
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
            std::cout << "USAGE : setktd <kp> <ki> <kd>" << std::endl;
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
            std::cout << "BAD VALUE" << std::endl;
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
            std::cout << "angle : " << i << std::endl;
            s.setAngle(i);
            usleep(1000*1000);
        }

        return 0;
    }

    else if(!args[0].compare("d"))
    {

        if(args.size() != 2)
        {
            std::cout << "USAGE : d <dist>" << std::endl;
            return 0;
        }

        long dist;
        try
        {
            dist = std::stol(args[1]);
        }
        catch (std::exception const &e)
        {
            std::cout << "BAD VALUE" << std::endl;
            return 0;
        }
#ifdef __arm__
        motion.orderTranslation(dist);
#endif
        return 0;
    }

    else if(!args[0].compare("c"))
    {
#ifdef __arm__
        std::cout << motion.getOdometry()->getLeftValue() << " ; " << motion.getOdometry()->getRightValue() << std::endl;
        std::cout << motion.getCurveRadius() << std::endl << std::endl;
#endif
        return 0;
    }

    else
    {
        std::cout << "No such command" << std::endl;
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

void serverWorker(void)
{
    int sockfd; // socket file descriptor
    struct sockaddr_in serv_addr;
    struct hostent *server;

    sockfd = socket(AF_INET, SOCK_STREAM, 0); // generate file descriptor
    if (sockfd < 0)
    {
        perror("ERROR opening socket");
        exit(0);
    }

    server = gethostbyname("localhost"); //the ip address (or server name) of the listening server.
    if (server == NULL)
    {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }

    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy(server->h_addr, (char *)&serv_addr.sin_addr.s_addr, (size_t) server->h_length);
    serv_addr.sin_port = htons(SOCKET_PORT);



    if (bind(sockfd,(struct sockaddr *)&serv_addr,sizeof(serv_addr)) < 0)
    {
        perror("ERROR binding");
        exit(0);
    }

    if(listen(sockfd, 1) <0)
    {
        perror("ERROR connecting");
        exit(0);
    }

    struct sockaddr_un client_name;
    socklen_t client_name_len=sizeof(struct sockaddr_un);
    int client_socket = accept(sockfd,(struct sockaddr *)&client_name, &client_name_len);

    if(client_socket < 0)
    {
        perror("ERROR opening client socket");
        exit(0);
    }

    std::string order = "";

    while(true)
    {
        char rbuff[256];
        ssize_t rbytes;

        //rbytes = read(sockfd, rbuff, sizeof(rbuff)); // read from socket and store the msg into buffer
        rbytes = recv(client_socket, rbuff, sizeof(rbuff), 0); // similar to read(), but return -1 if socket closed
        rbuff[rbytes] = '\0'; // set null terminal

        order = std::string(rbuff);

        if(treatOrder(order))
        {
#ifdef __arm__
            motion.stop();
#endif
            exit(0);
        }
    }
}

int writeMessage(int &sockfd, std::string &str)
{
    return (int) write(sockfd, str.c_str(), strlen(str.c_str()));
}