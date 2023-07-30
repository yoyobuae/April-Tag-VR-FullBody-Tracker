#if defined(__linux) || defined(__linux__) || defined(linux)
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

extern int h_errno;
#endif

#include "Tcp.hpp"

namespace Tcp {

    const int BUFSIZE = 1024;

#if defined(_WIN32) || defined(__WIN32__) || defined(WIN32)
    Server::Server() { }
    void Server::init(std::string name) { }
    Connection Server::accept()
    {
        return Connection();
    }

    Connection::Connection() { }
    Connection::~Connection() { }
    bool Connection::send(const char *buf, size_t len)
    {
        return false;
    }
    bool Connection::recv(char *buf, size_t len)
    {
        return false;
    }

    Client::Client(std::string name) : name(name) { }

    std::string Client::sendrecv(std::string buffer) { return std::string(); }

#elif defined(__linux) || defined(__linux__) || defined(linux)

    Server::Server() { }

    void Server::init(int portno)
    {
        struct sockaddr_in serv_addr;

        if ((listenfd = ::socket(AF_INET, SOCK_STREAM, 0)) == -1) {
            perror("socket");
        }

        bzero((char *) &serv_addr, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_addr.s_addr = INADDR_ANY;
        serv_addr.sin_port = htons(portno);

        if (::bind(listenfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) == -1) {
            perror("bind");
        }

        if (::listen(listenfd, 5) == -1) {
            perror("listen");
        }
    }

    Connection Server::accept()
    {
        struct sockaddr_in cli_addr;
        socklen_t clilen = sizeof(cli_addr);
        int connfd;
        if ((connfd = ::accept(listenfd, (struct sockaddr *)&cli_addr, &clilen)) == -1) {
            perror("accept");
        }
        return Connection(connfd);
    }

    Connection::Connection(int connfd) : connfd(connfd) { }

    Connection::~Connection()
    {
        ::close(connfd);
    }

    bool Connection::send(const char *buf, size_t len)
    {
        bool ret = true;

        if (::send(connfd, buf, len, 0) < 0) {
            perror("send");
            ret = false;
        }

        return ret;
    }

    bool Connection::recv(char *buf, size_t len)
    {
        bool ret = true;

        int n;
        n = ::recv(connfd, buf, len, 0);
        if (n < 0) {
            if (errno != EINTR)
            {
                perror("recv");
                ret = false;
            }
            buf[0] = '\0';
        }

        return ret;
    }

    Client::Client(std::string host, int port) : host(host), port(port), sockfd(-1) { }

    Client::~Client()
    {
        ::close(sockfd);
    }

    const char *gethostbyname_error_str()
    {
        switch(h_errno)
        {
        case HOST_NOT_FOUND: return "The specified host is unknown.";
        case NO_DATA: return "The  requested  name  is valid but does not have an IP address.  Another type of request to the name server for this domain may return an answer.";
        case NO_RECOVERY: return "A nonrecoverable name server error occurred.";
        case TRY_AGAIN: return "A temporary error occurred on an authoritative name server.  Try again later.";
        }
        return "(unknown)";
    }

    std::string Client::sendrecv(std::string buffer)
    {
        if (sockfd == -1) {
            // Create socket
            if ((sockfd = ::socket(AF_INET, SOCK_STREAM, 0)) == -1) {
                perror("socket");
                sockfd = -1;
                return std::string();
            }

            // Prepare address
            struct sockaddr_in serv_addr;
            struct hostent *server;

            server = gethostbyname(host.c_str());
            if (server == NULL) {
                fprintf(stderr, "gethostbyname: %s\n", gethostbyname_error_str());
                close(sockfd);
                sockfd = -1;
                return std::string();
            }
            bzero((char *) &serv_addr, sizeof(serv_addr));
            serv_addr.sin_family = AF_INET;
            bcopy((char *)server->h_addr, 
                    (char *)&serv_addr.sin_addr.s_addr,
                    server->h_length);
            serv_addr.sin_port = htons(port);

            // Connect
            if (::connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) == -1) {
                perror("connect");
                close(sockfd);
                sockfd = -1;
                return std::string();
            }
        }

        if (sockfd != -1) {
            // Send
            if (::send(sockfd, buffer.c_str(), buffer.size() + 1, 0) == -1) {
                perror("send");
                close(sockfd);
                sockfd = -1;
                return std::string();
            }

            // Receive
            char readBuf[BUFSIZE];
            int t;

            if ((t = ::recv(sockfd, readBuf, BUFSIZE, 0)) > 0) {
                readBuf[t] = '\0';
                return std::string(readBuf);
            }
            else {
                perror("recv");
                close(sockfd);
                sockfd = -1;
                return std::string();
            }
        }
        else
        {
            return std::string();
        }
    }

#else

    Server::Server() { }
    void Server::init(std::string name) { }
    Connection Server::accept()
    {
        return Connection();
    }

    Connection::Connection() { }
    Connection::~Connection() { }
    bool Connection::send(const char *buf, size_t len)
    {
        return false;
    }
    bool Connection::recv(char *buf, size_t len)
    {
        return false;
    }

    Client::Client(std::string name) : name(name) { }

    std::string Client::sendrecv(std::string buffer) { return std::string(); }

#endif

}; // namespace Tcp
