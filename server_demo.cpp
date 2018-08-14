/*
 * Copyright (c) 2016-2018 zzg@idealte.com.  All Rights Reserved.
 */

// system includes
#include <cmath>
#include <cstdbool>
#include <cstring>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <vector>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <string>
#include <sys/un.h>
#include <stddef.h>
#include <sys/time.h>
#include <inttypes.h>
#include <dirent.h>

using namespace std;

#define MAX_BUFF_LEN                            512
#define TMP_BUFF_LEN                            128

#define BROADCAST_PORT                          9936

int main(int argc, char* argv[])
{
    // Udp broadcast socket
    int server_udp_broadcast_socket;
    server_udp_broadcast_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (server_udp_broadcast_socket == -1)
    {
        printf("server_demo create socket errno=%d\n", errno);
        return 0;
    }

    // Set socket to be broadcast type
    int bOpt = 1;
    if (setsockopt(server_udp_broadcast_socket, SOL_SOCKET, SO_BROADCAST, &bOpt, sizeof(bOpt)) != 0)
    {
        printf("server_demo setsockopt errno=%d\n", errno);
        return 0;
    }

    struct sockaddr_in svr_sin;
    bzero(&svr_sin, sizeof(svr_sin));
    svr_sin.sin_family      = AF_INET;
    svr_sin.sin_port        = htons(BROADCAST_PORT);
    svr_sin.sin_addr.s_addr = htonl(INADDR_ANY);

    const int iMaxAttempts = 10;
    while (bind(server_udp_broadcast_socket, (struct sockaddr*)&svr_sin, sizeof(svr_sin)) != 0)
    {
        static int attempt_num = 0;
        printf("Attempt %d to bind server_udp_broadcast_socket failed!\n", attempt_num);

        if (attempt_num >= iMaxAttempts)
        {
            printf("Unable to bind server_udp_broadcast_socket after %d attempts.\n", attempt_num);
            return -1;
        }

        ++attempt_num;
        usleep(1e6);
    }

    while (true)
    {
        int nRecvSize = 0;
        int sin_size = sizeof(struct sockaddr_in);
        char buff[MAX_BUFF_LEN];

        nRecvSize = recvfrom(server_udp_broadcast_socket, buff, MAX_BUFF_LEN, 0, (struct sockaddr *)&svr_sin, (socklen_t*)&sin_size);

        if (nRecvSize > 0)
        {
            buff[nRecvSize] = '\0';
            char *pIPAddr = inet_ntoa(svr_sin.sin_addr);
            int nPort = ntohs(svr_sin.sin_port);
            printf("recvfrom client: buff:%s, IpAddr: %s port: %d\n" , buff, pIPAddr, nPort);

            memset(buff, 0, sizeof(buff));
            strcpy(buff, "server msg");

            int nSendSize = sendto(server_udp_broadcast_socket, buff, strlen(buff), 0, (struct sockaddr *)&svr_sin, sin_size);
            printf("sendto client: buff:%s\n" , buff);
        }

        sleep(1);
    }
}

