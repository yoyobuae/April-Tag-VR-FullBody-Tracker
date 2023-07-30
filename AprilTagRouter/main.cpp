#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Ipc.hpp"
#include "Tcp.hpp"

Ipc::Server ipcServer;
Tcp::Server tcpServer;

void ipcToTcp(std::string ipcPipeName, std::string tcpHost, int tcpPort)
{
    ipcServer.init(ipcPipeName);

    char buffer[1024];

    for (;;) 
    {
        Ipc::Connection ipcConnection = ipcServer.accept();

        if (ipcConnection.recv(buffer, sizeof(buffer)))
        {
            std::string request = buffer;

            Tcp::Client tcpClient(tcpHost, tcpPort);

            std::string response = tcpClient.sendrecv(request);

            ipcConnection.send(response.c_str(), (response.length() + 1));
        }
    }

}

void tcpToIpc(int tcpPort, std::string ipcPipeName)
{
    tcpServer.init(tcpPort);

    char buffer[1024];

    for (;;) 
    {
        Tcp::Connection tcpConnection = tcpServer.accept();

        if (tcpConnection.recv(buffer, sizeof(buffer)))
        {
            std::string request = buffer;

            Ipc::Client ipcClient(ipcPipeName);

            std::string response = ipcClient.sendrecv(request);

            tcpConnection.send(response.c_str(), (response.length() + 1));
        }
    }

}

void print_tcp2ipc_usage(char *program_name)
{
        printf("%s --tcp2ipc <tcpPort> <ipcPipeName>\n", program_name);
}

void print_ipc2tcp_usage(char *program_name)
{
        printf("%s --ipc2tcp <ipcPipeName> <tcpHost> <tcpPort>\n", program_name);
}

void print_usage(char *program_name)
{
        printf("%s --ipc2tcp <ipcPipeName> <tcpHost> <tcpPort>\n", program_name);
        printf("or\n");
        printf("%s --tcp2ipc <tcpPort> <ipcPipeName>\n", program_name);
}

int main(int argc, char *argv[])
{
    if (argc <= 1)
    {
        print_usage(argv[0]);
        exit(1);
    }

    printf("%d\n", argc);

    if (strcmp(argv[1], "--ipc2tcp") == 0)
    {
        if (argc != 5)
        {
            print_ipc2tcp_usage(argv[0]);
            exit(1);
        }

        std::string ipcPipeName = argv[2];
        std::string tcpHost = argv[3];
        int tcpPort = atoi(argv[4]);

        ipcToTcp(ipcPipeName, tcpHost, tcpPort);
    }
    else if (strcmp(argv[1], "--tcp2ipc") == 0)
    {
        if (argc != 4)
        {
            print_tcp2ipc_usage(argv[0]);
            exit(1);
        }

        int tcpPort = atoi(argv[2]);
        std::string ipcPipeName = argv[3];

        tcpToIpc(tcpPort, ipcPipeName);
    }
    else
    {
        print_usage(argv[0]);
        exit(1);
    }

    return 0;
}
