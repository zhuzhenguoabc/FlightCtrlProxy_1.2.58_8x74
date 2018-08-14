/*
 * Copyright (c) 2015-2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
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

using namespace std;

#define STR_SEPARATOR                           ","

#define SNAV_TASK_SNAV_UPDATE                   "8008"
#define SNAV_TASK_SNAV_UPDATE_RETURN            "9008"

#define OTA_UDP_PORT                        14888
#define SNAV_UPDATE_PORT                    14999

#define MAX_BUFF_LEN                        512

#define OTA_LINARO_PATH                     "/tmp/update-linaro.zip"
#define OTA_SNAV_PATH                       "/tmp/update-snav.zip"
#define OTA_RESTART_SNAV                    "restart_snav"


#define OPEN_GPS                            "open_gps"
#define CLOSE_GPS                           "close_gps"


#ifdef __DEBUG
#define DEBUG(format, ...) printf(format, ##__VA_ARGS__)
#else
#define DEBUG(format,...)
#endif


struct timeval timeout_ota_udp = {0, 300000};           //300ms

vector<string> split(const string& s, const string& delim)
{
    vector<string> elems;

    size_t pos = 0;
    size_t len = s.length();
    size_t delim_len = delim.length();

    if (delim_len == 0)
    {
        return elems;
    }

    while (pos < len)
    {
        int find_pos = s.find(delim, pos);
        if (find_pos < 0)
        {
            elems.push_back(s.substr(pos, len - pos));
            break;
        }
        elems.push_back(s.substr(pos, find_pos - pos));
        pos = find_pos + delim_len;
    }

    return elems;
}


void* ThreadEmergencySnavUpdate(void*)
{
    DEBUG("ThreadEmergencySnavUpdate start\n");

    //udp
    int server_udp_sockfd;
    int server_udp_len;
    struct sockaddr_in server_udp_address;

    server_udp_address.sin_family = AF_INET;
    server_udp_address.sin_addr.s_addr = htonl(INADDR_ANY);
    server_udp_address.sin_port = htons(SNAV_UPDATE_PORT);
    server_udp_len = sizeof(server_udp_address);

    server_udp_sockfd = socket(AF_INET,SOCK_DGRAM,0);

    /*
    char log_filename[256] = "/home/linaro/log_ota";
    // Confirm logfile name end

    freopen(log_filename, "a", stdout);
    setbuf(stdout, NULL);       //needn't cache and fflush, output immediately
    freopen(log_filename, "a", stderr);
    setbuf(stderr, NULL);       //needn't cache and fflush, output immediately
    */

    const int kMaxNumAttempts = 10;
    while (bind(server_udp_sockfd, (struct sockaddr*)&server_udp_address, server_udp_len) != 0)
    {
        static int attempt_number = 0;
        DEBUG("Attempt %d to bind udp failed!\n", attempt_number);

        if (attempt_number >= kMaxNumAttempts)
        {
            DEBUG("Unable to bind after %d attempts.\n", attempt_number);
            return 0;
        }

        ++attempt_number;
        usleep(1e6);
    }

    while (true)
    {
        int length = 0;
        struct sockaddr_in remote_addr;
        int sin_size = sizeof(struct sockaddr_in);
        char udp_buff_data[MAX_BUFF_LEN];

        //receive the udp data
        length = recvfrom(server_udp_sockfd,udp_buff_data,MAX_BUFF_LEN,0, (struct sockaddr *)&remote_addr,(socklen_t*)&sin_size);

        if (length > 0)
        {
            char result_to_client[MAX_BUFF_LEN];
            string recv_udp_cmd;
            vector<string> udp_msg_array;

            recv_udp_cmd = udp_buff_data;
            udp_msg_array = split(recv_udp_cmd, STR_SEPARATOR);

            DEBUG("EmergencySnavUpdate received data from %s,%d:\n", inet_ntoa(remote_addr.sin_addr),  ntohs(remote_addr.sin_port));
            udp_buff_data[length] = '\0';
            DEBUG("EmergencySnavUpdate get data udp_buff_data=%s\n", udp_buff_data);

            if ((udp_msg_array.size() >= 1)
                && (udp_msg_array[0].compare(SNAV_TASK_SNAV_UPDATE) == 0))
            {
                memset(result_to_client, 0, MAX_BUFF_LEN);
                sprintf(result_to_client, "%s", SNAV_TASK_SNAV_UPDATE_RETURN);

                length = sendto(server_udp_sockfd, result_to_client, strlen(result_to_client), 0,
                                    (struct sockaddr*)&remote_addr, sizeof(struct sockaddr));
                DEBUG("SNAV_TASK_SNAV_UPDATE_RETURN result_to_client=%s, length=%d\n", result_to_client, length);

                system("chmod 777 /tmp/update-snav.zip");

                system("rm -rf /tmp/update-snav/");
                system("chmod 777 /tmp/update-snav.zip");
                system("tar xvf /tmp/update-snav.zip -C /tmp/");    //zxvf
                system("chmod -R 777 /tmp/update-snav/");
                system("/tmp/update-snav/update.sh");
            }
        }
        else
        {
            DEBUG("EmergencySnavUpdate return length=%d, errno=%d\n", length, errno);
        }
    }
}


int main(int argc, char* argv[])
{
    // Create the Emergency snav update
    bool snav_update_flag = false;
    while (!snav_update_flag)
    {
        pthread_t emergency_snav_update_thread;
        pthread_attr_t thread_attr;
        int result;

        result = pthread_attr_init(&thread_attr);
        if (result != 0)
        {
            perror("emergency_snav_update_thread Attribute init failed");
            continue;
        }

        result = pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_DETACHED);
        if (result != 0)
        {
            perror("emergency_snav_update_thread Setting detached attribute failed");
            pthread_attr_destroy(&thread_attr);
            continue;
        }

        result = pthread_create(&emergency_snav_update_thread, &thread_attr, ThreadEmergencySnavUpdate, NULL);
        if (result != 0)
        {
            perror("ThreadEmergencySnavUpdate create failed");
            pthread_attr_destroy(&thread_attr);
            continue;
        }

        snav_update_flag = true;
        pthread_attr_destroy(&thread_attr);
    }

    //udp
    int server_udp_sockfd;
    int server_udp_len;
    struct sockaddr_in server_udp_address;

    server_udp_address.sin_family = AF_INET;
    server_udp_address.sin_addr.s_addr = htonl(INADDR_ANY);
    server_udp_address.sin_port = htons(OTA_UDP_PORT);
    server_udp_len = sizeof(server_udp_address);

    server_udp_sockfd = socket(AF_INET,SOCK_DGRAM,0);

    int bind_result = bind(server_udp_sockfd, (struct sockaddr*)&server_udp_address, server_udp_len);

    //300MS avoid of udp missing data
    setsockopt(server_udp_sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout_ota_udp, sizeof(struct timeval));
    setsockopt(server_udp_sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout_ota_udp, sizeof(struct timeval));

    while (true)
    {
        int length = 0;
        struct sockaddr_in remote_addr;
        int sin_size = sizeof(struct sockaddr_in);
        char udp_buff_data[MAX_BUFF_LEN];

        //receive the udp data
        length = recvfrom(server_udp_sockfd,udp_buff_data,MAX_BUFF_LEN-1,0, (struct sockaddr *)&remote_addr,(socklen_t*)&sin_size);

        if (length > 0)
        {
            DEBUG("udp recvfrom received data from %s,%d:\n", inet_ntoa(remote_addr.sin_addr),  ntohs(remote_addr.sin_port));
            udp_buff_data[length] = '\0';
            DEBUG("udp recvfrom get data udp_buff_data=%s\n", udp_buff_data);

            if (strcmp(udp_buff_data, OTA_LINARO_PATH) == 0)
            {
                system("stop snav");
                system("install-update /tmp/update-linaro.zip");
            }
            else if (strcmp(udp_buff_data, OTA_SNAV_PATH) == 0)
            {
                system("rm -rf /tmp/update-snav/");
                system("chmod 777 /tmp/update-snav.zip");
                system("tar xvf /tmp/update-snav.zip -C /tmp/");    //zxvf
                system("chmod -R 777 /tmp/update-snav/");
                system("/tmp/update-snav/update.sh");
            }
            else if (strcmp(udp_buff_data, OTA_RESTART_SNAV) == 0)
            {
                system("stop snav");
                //sleep(3);
                sleep(8);
                system("start snav");
            }
#if 0
            else if (strcmp(udp_buff_data, OPEN_GPS) == 0)
            {
                system("stop snav");
                sleep(5);
                system("sed -i 's/\"gps_port\".*/\"gps_port\" value=\"4\"\\/\\>/g'  /usr/share/data/adsp/eagle_default_port_mapping.xml");
                system("sed -i 's/\"gps_type\".*/\"gps_type\" value=\"1\"\\/\\>/g'  /usr/share/data/adsp/eagle_default_port_mapping.xml");
                system("sed -i 's/\"compass_port\".*/\"compass_port\" value=\"2\"\\/\\>/g'  /usr/share/data/adsp/eagle_default_port_mapping.xml");
                sleep(2);
                system("start snav");
            }
            else if (strcmp(udp_buff_data, CLOSE_GPS) == 0)
            {
                system("stop snav");
                sleep(5);
                system("sed -i 's/\"gps_port\".*/\"gps_port\" value=\"-1\"\\/\\>/g'  /usr/share/data/adsp/eagle_default_port_mapping.xml");
                system("sed -i 's/\"gps_type\".*/\"gps_type\" value=\"-1\"\\/\\>/g'  /usr/share/data/adsp/eagle_default_port_mapping.xml");
                system("sed -i 's/\"compass_port\".*/\"compass_port\" value=\"-1\"\\/\\>/g'  /usr/share/data/adsp/eagle_default_port_mapping.xml");
                sleep(2);
                system("start snav");
            }
#endif
        }
        else
        {
            //DEBUG("udp recvfrom return length=%d, errno=%d\n", length, errno);
        }
    }

    //fclose(stdout);
    //fclose(stderr);

    return 0;
}

