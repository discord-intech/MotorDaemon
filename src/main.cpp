#include <iostream>
#include <vector>
#include <sys/types.h>
#include <sys/un.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <syslog.h>
#include <netdb.h>
#include <signal.h>
#include <stdlib.h>
#include "../include/Selector.hpp"

#define SOCKET_PORT 56987
#define BUFFER_MAX_SIZE 2048
#define END_OF_ORDER '\r'
#define SERVER_MODE_CMD "-s"
#define DAEMON_NAME "motordaemon"

void serverWorker(void);
void localWorker(void);
char * readOrder(int socket);

int sockfd;
std::thread t;

void signalHandler(int sign)
{
    if(sign == SIGINT)
    {
        t.detach();
        pthread_cancel(t.native_handle());
        close(sockfd);
        exit(0);
    }
}

class Writters
{
public:
    static void writeMessage(int &sockfd, char * str)
    {
        if(strlen(str) > BUFFER_MAX_SIZE)
        {
            printf("RESPONSE TOO LONG : %s\n", str);
            return;
        }

        write(sockfd, str, strlen(str));
    }

    static void printMessage(char * str)
    {
        std::cout << str << std::endl;
    }
};

int main(int argc, char *argv[])
{

    if (signal(SIGINT, signalHandler) == SIG_ERR)
    {
        std::cerr << std::endl << "Can't catch SIGINT" << std::endl;
    }

#ifdef __arm__
    motion = MotionController();
    motion.init();
#endif

    if(argc >= 2 && !strcmp(argv[1], SERVER_MODE_CMD))
    {
        setlogmask(LOG_UPTO(LOG_NOTICE));
        openlog(DAEMON_NAME, LOG_CONS | LOG_NDELAY | LOG_PERROR | LOG_PID, LOG_USER);
        serverMode = true;
        t = std::thread(serverWorker);
        syslog(LOG_INFO, "MotorDaemon launched in server mode");
        t.join(); //Do not shut down the main thread
    }
    else
    {
        t = std::thread(localWorker);
        std::cout << "MotorDaemon launched in bash mode" << std::endl;
#ifdef __arm__
        std::cout << "ARM CPU detected, using PWMs" << std::endl;
#endif
        t.join(); //Do not shut down the main thread
    }

    return 0;
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

        if(treatOrder(order, &Writters::printMessage))
        {
#ifdef __arm__
            motion.stop();
#endif
            return;
        }
    }

}

void serverWorker(void)
{
    struct sockaddr_in serv_addr;
    struct hostent *server;

    sockfd = socket(AF_INET, SOCK_STREAM, 0); // generate file descriptor
    if (sockfd < 0)
    {
        perror("ERROR opening socket");
        return;
    }

  /* server = gethostbyname("localhost"); //the ip address (or server name) of the listening server.
    if (server == NULL)
    {
        fprintf(stderr,"ERROR, no such host\n");
        close(sockfd);
        return;
    }*/

    memset((char *) &serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl (INADDR_ANY);
   // memcpy(server->h_addr, (char *)&serv_addr.sin_addr.s_addr, (size_t) server->h_length);
    serv_addr.sin_port = htons(SOCKET_PORT);



    if (bind(sockfd,(struct sockaddr *)&serv_addr,sizeof(serv_addr)) < 0)
    {
        perror("ERROR binding");
        close(sockfd);
        return;
    }

    listening:if(listen(sockfd, 1) <0)
    {
        perror("ERROR connecting");
        close(sockfd);
        return;
    }

    struct sockaddr_un client_name;
    socklen_t client_name_len=sizeof(struct sockaddr_un);
    int client_socket = accept(sockfd,(struct sockaddr *)&client_name, &client_name_len);

    if(client_socket < 0)
    {
        perror("ERROR opening client socket");
        close(sockfd);
        return;
    }

    std::string order = "";

    while(true)
    {
        char * rbuff;

        rbuff = readOrder(client_socket);

        if(rbuff == 0)
        {
            perror("ERROR socket is unavailable");
            close(client_socket);
            goto listening;
        }

        order = std::string(rbuff);

        if(treatOrder(order, std::bind(&Writters::writeMessage, client_socket, std::placeholders::_1)))
        {
#ifdef __arm__
            motion.stop();
#endif
            close(client_socket);
            close(sockfd);
            return;
        }
    }
}

char * readOrder(int socket)
{
    char * buf = (char*)malloc(BUFFER_MAX_SIZE * sizeof(char));

    memset(buf, 0, BUFFER_MAX_SIZE*sizeof(char));

    char * actual = (char*)malloc(2*sizeof(char));

    memset(actual, 0, sizeof(actual));

    ssize_t bytes;

    while(true)
    {
        bytes = recv(socket, actual, 1, 0);

        if(bytes < 0)
        {
            return 0;
        }

        if(actual[0] == END_OF_ORDER) break;

        actual[1] = 0;

        strcat(buf, actual);

        memset(actual, 0, sizeof(actual));
    }

    return buf;
}

