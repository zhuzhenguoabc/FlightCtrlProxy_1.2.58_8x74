/*
 * Copyright (c) 2016-2017 zzg@idealte.com.  All Rights Reserved.
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
//#include <mysql/mysql.h>


// Waypoint utilities
#include "snav_waypoint_utils.hpp"
// Snapdragon Navigator
#include "snapdragon_navigator.h"

using namespace std;

#ifdef __DEBUG
#define DEBUG(format, ...) printf(format, ##__VA_ARGS__)
#else
#define DEBUG(format, ...)
#endif

typedef unsigned char byte;

// LED COLOR
enum class LedColor
{
  UNKNOWN,
  LED_COLOR_RED,
  LED_COLOR_GREEN,
  LED_COLOR_BLUE,
  LED_COLOR_WHITE,
  LED_COLOR_YELLOW,
  LED_COLOR_PURPLE,
  LED_COLOR_BLUE_EX
};


// Global variables
static LedColor led_color_status = LedColor::UNKNOWN;
static bool bNeedLedColorCtl = false;

#define FALSE  -1
#define TRUE   0

/*******************led control************************************/
int main(int argc, char* argv[])
{
    DEBUG("ThreadLedControl start\n");

    uint8_t led_colors[3] = {0, 0, 0};  //R, G, B
    int32_t timeout = 1000000;          // 1S, timeout for flight controller to take over LED control after API commands stop

    SnavCachedData* snav_data = NULL;
    if (sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_data) != 0)
    {
        DEBUG("\nFailed to get flight data pointer!\n");
        return -1;
    }

    if (argc >= 2)
    {
        while (true)
        {
            if (sn_update_data() != 0)
            {
                printf("sn_update_data failed!\n");
            }
            else
            {
                if (argc >= 4)
                {
                    led_colors[0] = atoi(argv[1]);
                    led_colors[1] = atoi(argv[2]);
                    led_colors[2] = atoi(argv[3]);

                    printf("*********%s,%s,%s*********\n", argv[1],argv[2],argv[3]);
                }
                else if (argc == 3)
                {
                    led_colors[0] = atoi(argv[1]);
                    led_colors[1] = atoi(argv[2]);

                    printf("*********%s,%s*********\n", argv[1],argv[2]);
                }
                else if (argc == 2)
                {
                    led_colors[0] = atoi(argv[1]);

                    printf("*********%s*********\n", argv[1]);
                }

                int ret = sn_set_led_colors(led_colors, sizeof(led_colors), timeout);

                if (ret != 0)
                {
                    printf("sn_set_led_colors returned %d\n",ret);
                }

                usleep(10000);  // 10ms note that commands should only be sent as often as needed (minimize message traffic)
            }
        }
    }
    else
    {
        printf("plese run as: led_light_test 255 255 0 \n");
    }

}
