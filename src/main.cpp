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
#include <Settings.hpp>
#include <malloc.h>
#include "../include/Selector.hpp"

#define SOCKET_PORT 56987
#define PROXY_SOCKET_PORT 56990
#define BUFFER_MAX_SIZE 1024
#define SERVER_MODE_CMD "-s"
#define PROXY_MODE_CMD "-p"
#define DAEMON_NAME "motordaemon"

void serverWorker(void);
void localWorker(void);
void proxyWorker(void);
char * readOrder(int socket);

int sockfd;
std::thread t;
char * proxyAdress;

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
            printf("RESPONSE TOO LONG (cmb): %s\n", str);
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
    else if(argc == 3 && !strcmp(argv[1], PROXY_MODE_CMD))
    {
        setlogmask(LOG_UPTO(LOG_NOTICE));
        openlog(DAEMON_NAME, LOG_CONS | LOG_NDELAY | LOG_PERROR | LOG_PID, LOG_USER);
        proxyMode = true;
        proxyAdress = argv[2];
        t = std::thread(proxyWorker);
        syslog(LOG_INFO, "MotorDaemon launched in proxy mode");
        t.join(); //Do not shut down the main thread
    }
    else if(argc == 2 && !strcmp(argv[1], PROXY_MODE_CMD))
    {
        setlogmask(LOG_UPTO(LOG_NOTICE));
        openlog(DAEMON_NAME, LOG_CONS | LOG_NDELAY | LOG_PERROR | LOG_PID, LOG_USER);
        proxyMode = true;
        proxyAdress = (char *)settings.get("IP_MOTORDAEMONPROXY").c_str();
        t = std::thread(proxyWorker);
        syslog(LOG_INFO, "MotorDaemon launched in proxy mode");
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

    ssize_t rbytes;

    char rbuffv[12];
    rbytes = recv(client_socket, rbuffv, sizeof(rbuffv), 0); // similar to read(), but return -1 if socket closed
    rbuffv[11] = 0;

    if(rbytes < 0 || strcmp(rbuffv, "motordaemon"))
    {
        printf("Wrong app connected to client socket\n");
        close(client_socket);
        goto listening;
    }

    std::string order = "";

    while(true)
    {
        char * rbuff = (char *)malloc(sizeof(char)*BUFFER_MAX_SIZE);

        rbytes = recv(client_socket, rbuff, sizeof(rbuff), 0); // similar to read(), but return -1 if socket closed

        if(rbytes < 0)
        {
            perror("ERROR socket is unavailable");
            close(client_socket);
            goto listening;
        }

        rbuff[rbytes] = '\0'; // set null terminal

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
        free(rbuff); rbuff = NULL;
    }
}

void connecting ( in_port_t port, const char * hostname )
{
    sockfd = socket ( AF_INET, SOCK_STREAM, 0 ) ;
    if ( sockfd == -1 ) {
        perror ( "socket" ) ;
    }

    // Search for the host name.
    struct hostent * hostent ;
    hostent = gethostbyname ( hostname ) ;
    if ( ! hostent ) {
        fprintf ( stderr, "Problem with 'gethostbyname'.\n" ) ;
    }

    // Initialisation of the sockaddr_in data structure.
    struct sockaddr_in addr ;
    memset ( & addr, 0, sizeof ( struct sockaddr_in ) ) ;
    addr.sin_family = AF_INET ;
    addr.sin_port = port;
    addr.sin_addr.s_addr = ( ( struct in_addr * ) ( hostent -> h_addr ) ) -> s_addr ;

    // Name the socket.
    int code ;
    code = connect ( sockfd, ( struct sockaddr * ) & addr, sizeof ( struct sockaddr_in ) ) ;
    if ( code == -1 ) {
        perror ( "connect" ) ;
    }
}

void proxyWorker(void)
{
    connect:connecting (htons(PROXY_SOCKET_PORT), proxyAdress) ;

    printf("MotorDaemonProxy detected on %s:%d\nMotorDaemon is ready\n", proxyAdress, PROXY_SOCKET_PORT);

    std::string order;

    ssize_t rbytes;

    rbytes = write(sockfd, "motordaemon", 11);

    if(rbytes < 0)
    {
        perror("ERROR socket is unavailable");
        close(sockfd);
        goto connect;
    }

    for( ; ; )
    {
        char * rbuff = (char *)malloc(sizeof(char)*BUFFER_MAX_SIZE);

        rbytes = recv(sockfd, rbuff, sizeof(char)*BUFFER_MAX_SIZE, 0); // similar to read(), but return -1 if socket closed

        if(rbytes < 0)
        {
            perror("ERROR socket is unavailable");
            close(sockfd);
            goto connect;
        }

        rbuff[rbytes] = '\0'; // set null terminal

        order = std::string(rbuff);

        if(treatOrder(order, std::bind(&Writters::writeMessage, sockfd, std::placeholders::_1)))
        {
#ifdef __arm__
            motion.stop();
#endif
            close(sockfd);
            return;
        }
        free(rbuff); rbuff = NULL;
    }
}
