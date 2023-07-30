#if defined(__linux) || defined(__linux__) || defined(linux)
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
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
            perror("recv");
            ret = false;
        }

        return ret;
    }

    Client::Client(std::string host, int port) : host(host), port(port) { }

    std::string Client::sendrecv(std::string buffer)
    {
        // Create socket
        int sockfd;

        if ((sockfd = ::socket(AF_INET, SOCK_STREAM, 0)) == -1) {
            perror("socket");
            return std::string();
        }

        // Prepare address
	struct sockaddr_in serv_addr;

	bzero((char *) &serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(port);

        // Connect
        if (::connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) == -1) {
            perror("connect");
            close(sockfd);
            return std::string();
        }

        // Send
        if (::send(sockfd, buffer.c_str(), buffer.size() + 1, 0) == -1) {
            perror("send");
            close(sockfd);
            return std::string();
        }

        // Receive
        char readBuf[BUFSIZE];
        int t;

        if ((t = ::recv(sockfd, readBuf, BUFSIZE, 0)) > 0) {
            readBuf[t] = '\0';
            close(sockfd);
            return std::string(readBuf);
        }
        else {
            perror("recv");
            close(sockfd);
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
