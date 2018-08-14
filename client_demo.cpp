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

// snav head files
#include "snav_waypoint_utils.hpp"
#include "snapdragon_navigator.h"

using namespace std;


#define MAX_BUFF_LEN                            512
#define TMP_BUFF_LEN                            128

#define BROADCAST_PORT                          9936

int main(int argc, char* argv[])
{
    // Udp broadcast socket
    int client_udp_broadcast_socket;
    client_udp_broadcast_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (client_udp_broadcast_socket == -1)
    {
        printf("client_demo create socket errno=%d\n", errno);
        return 0;
    }

    // Set socket to be broadcast type
    int bOpt = 1;
    if (setsockopt(client_udp_broadcast_socket, SOL_SOCKET, SO_BROADCAST, &bOpt, sizeof(bOpt)) != 0)
    {
        printf("client_demo setsockopt errno=%d\n", errno);
        return 0;
    }

    struct sockaddr_in client_sin;
    bzero(&client_sin, sizeof(client_sin));
    client_sin.sin_family      = AF_INET;
    client_sin.sin_port        = htons(BROADCAST_PORT);
    client_sin.sin_addr.s_addr = htonl(INADDR_BROADCAST);

    while (true)
    {
        int sin_size = sizeof(struct sockaddr_in);
        char buff[MAX_BUFF_LEN];

        memset(buff, 0, sizeof(buff));
        strcpy(buff, "client msg");

        int nSendSize = sendto(client_udp_broadcast_socket, buff, strlen(buff), 0, (struct sockaddr *)&client_sin, sin_size);

        char *pIPAddr = inet_ntoa(client_sin.sin_addr);
        int nPort = ntohs(client_sin.sin_port);

        printf("sendbroadcast buff:%s, IpAddr: %s port: %d\n" , buff, pIPAddr, nPort);

        memset(buff, 0, sizeof(buff));
        int nRecvSize = recvfrom(client_udp_broadcast_socket, buff, MAX_BUFF_LEN, 0, (struct sockaddr *)&client_sin, (socklen_t*)&sin_size);

        if (nRecvSize > 0)
        {
            buff[nRecvSize] = '\0';
            char *pIPAddr = inet_ntoa(client_sin.sin_addr);
            int nPort = ntohs(client_sin.sin_port);

            printf("recvfrom server: buff:%s, IpAddr: %s port: %d\n" , buff, pIPAddr, nPort);
        }

        sleep(1);
    }
}

