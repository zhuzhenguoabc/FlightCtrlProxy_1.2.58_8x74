/*
 * Copyright (c) 2015-2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>


#include "snapdragon_navigator.h"

#define CAM_ERROR       "Camera not found"

#define CFG_FILE_PATH   "/usr/bin/calibration_cfg.txt"
#define CFG_NUMBERS     9

#define UART_DEVICE     "/dev/ttyHSL2"

int speed_arr[] = {B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300};
int name_arr[]  = { 115200,  38400,  19200,  9600,  4800,  2400,  1200,  300};
int fd;
static unsigned char  read_buf[128];
static unsigned char  cont_buf[128];



void print_sn_calib_status_string(SnCalibStatus status)
{
  if (status == SN_CALIB_STATUS_NOT_CALIBRATED)
    printf("SN_CALIB_STATUS_NOT_CALIBRATED\n");
  else if (status == SN_CALIB_STATUS_CALIBRATION_IN_PROGRESS)
    printf("SN_CALIB_STATUS_CALIBRATION_IN_PROGRESS\n");
  else if (status == SN_CALIB_STATUS_CALIBRATED)
    printf("SN_CALIB_STATUS_CALIBRATED\n");
  else
    printf("invalid code\n");
}

void print_sn_data_status_string(SnDataStatus status)
{

  if (status == SN_DATA_UNCALIBRATED)
    printf("SN_DATA_UNCALIBRATED\n");
  else if (status == SN_DATA_WARNING)
    printf("SN_DATA_WARNING\n");
  else if (status == SN_DATA_VALID)
    printf("SN_DATA_VALID\n");
  else
    printf("SN_DATA_INVALID\n");
}


/*set serial port transport speed*/
void set_speed(int fd, int speed)
{
    int   i;
    int   status;
    int   size;
    struct termios   Opt;
    tcgetattr(fd, &Opt);

    size = sizeof(speed_arr)/sizeof(int);

    for (i = 0; i < size; i++)
    {
        if (speed == name_arr[i])
        {
            tcflush(fd, TCIOFLUSH);
            cfsetispeed(&Opt, speed_arr[i]);
            cfsetospeed(&Opt, speed_arr[i]);
            status = tcsetattr(fd, TCSANOW, &Opt);

            if (status != 0)
            {
                printf("tcsetattr fd1");
                return;
            }
            tcflush(fd, TCIOFLUSH);
        }
    }
}

/*set serial port databits(7/8), stopbits(1/2), paritybit(N/E/O/S)*/
int set_Parity(int fd, int databits, int stopbits, int parity)
{
    struct termios options;
    if (tcgetattr(fd,&options) != 0)
    {
        printf("SetupSerial 1");
        return(0);
    }
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr,"Unsupported data size\n"); return (0);
    }
    switch (parity)
    {
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB;
            options.c_iflag &= ~INPCK;
            break;
        case 'o':
        case 'O':
            options.c_cflag |= (PARODD | PARENB);
            options.c_iflag |= INPCK;
            break;
        case 'e':
        case 'E':
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            options.c_iflag |= INPCK;
            break;
        case 'S':
        case 's':  /*as no parity*/
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;break;
        default:
            fprintf(stderr,"Unsupported parity\n");
            return (0);
        }

    switch (stopbits)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
           break;
        default:
             fprintf(stderr,"Unsupported stop bits\n");
             return (0);
    }
    /* Set input parity option */
    if (parity != 'n')
        options.c_iflag |= INPCK;
    tcflush(fd,TCIFLUSH);
    options.c_cc[VTIME] = 150;
    options.c_cc[VMIN] = 0;
    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        perror("SetupSerial 3");
        return (0);
    }
    options.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/
    options.c_oflag  &= ~OPOST;   /*Output*/
    return (1);
}


void  init_uart()
{
    printf("Start  init UART ...\n");
    fd = open(UART_DEVICE, O_RDWR|O_NONBLOCK);

    if (fd < 0) {
        perror(UART_DEVICE);
        printf("perror(UART_DEVICE)...\n");
    }

    printf("Open...\n");
    set_speed(fd,115200);
    if (set_Parity(fd,8,1,'N') == 0)  {
        printf("Set Parity Error\n");
    }
}



int main(int argc, char* argv[])
{
	int i;
	char name[10];
	int  value;

    int imu_switch = 0;
    int baro_switch = 0;
    int mag_switch = 0;
    int gps_switch = 0;
    int sonar_switch = 0;
    int optic_flow_switch = 0;
    int esc_switch = 0;
    int cam_switch = 0;
    int infrared_switch = 0;

    int cam_value = 1;
    int infrared_value = 1;

	int count = 0;
	int infrared_size = 0;
	float	ir_distance = 0;

	char  buf[256];

    FILE *fpRead=fopen(CFG_FILE_PATH, "a+");

    if(fpRead != NULL)
    {
        for(i=0;i<CFG_NUMBERS;i++)
        {
            if (fscanf(fpRead, "%s%d", name, &value) != EOF)
            {
                if ((strcmp(name, "imu") == 0))
                {
                    if (value == 1)
                    {
                        imu_switch = 1;
                    }
                }
                else if ((strcmp(name, "baro") == 0))
                {
                    if (value == 1)
                    {
                        baro_switch = 1;
                    }
                }
                else if ((strcmp(name, "mag") == 0))
                {
                    if (value == 1)
                    {
                        mag_switch = 1;
                    }
                }
                else if ((strcmp(name, "gps") == 0))
                {
                    if (value == 1)
                    {
                        gps_switch = 1;
                    }
                }
                else if ((strcmp(name, "sonar") == 0))
                {
                    if (value == 1)
                    {
                        sonar_switch = 1;
                    }
                }
                else if ((strcmp(name, "optic_flow") == 0))
                {
                    if (value == 1)
                    {
                        optic_flow_switch = 1;
                    }
                }
                else if ((strcmp(name, "esc") == 0))
                {
                    if (value == 1)
                    {
                        esc_switch = 1;
                    }
                }
                else if ((strcmp(name, "cam") == 0))
                {
                    if (value == 1)
                    {
                        cam_switch = 1;
                    }
                }
                else if ((strcmp(name, "infrared") == 0))
                {
                    if (value == 1)
                    {
                        infrared_switch = 1;
                    }
                }
            }
            else
            {
                printf("fscanf == EOF!!!\n");
            }
        }
        fclose(fpRead);
    }
    else
    {
        printf("fpRead == NULL!!!\n");
    }

	printf("\n*****START*******\n");

	init_uart();

  SnavCachedData* snav_data = NULL;
  if (sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_data) != 0)
  {
    printf("\nFailed to get flight data pointer!\n");
    return -1;
  }

  // put vehicle into infinite loop
  int loop_counter = 0;
  unsigned int ii = 0;
  int pass_false_flag = 0;

	// Check the cam status
    if (cam_switch == 1)
    {
        char get_cam_status_cmd[128] = "camera-test -t 3 -f hires | grep 'Camera not found'";
        char cam_status[64];

		memset(cam_status, 0, 64);

        FILE *fp = popen(get_cam_status_cmd, "r");
        fgets(cam_status, sizeof(cam_status), fp);
        pclose(fp);

        printf("camera-test strlen(cam_status)=%d\n", strlen(cam_status));
        printf("camera-test result=%s\n", cam_status);

        if (strlen(cam_status) > 1)
        {
            cam_value = 0;
        }
        else
        {
            cam_value = 1;
        }

        printf("cam_value=%d\n", cam_value);
    }
    // Check End

  for(loop_counter=0;;loop_counter++)
  {
	sleep(5);
	system("clear");
	pass_false_flag = 0;

	// Check the infrared status
    if (infrared_switch == 1)
    {
		count = 0;

		memset(buf, 0, 256);
        memset(read_buf, 0, 128);
        memset(cont_buf, 0, 128);

		tcflush(fd, TCIOFLUSH);

		while (count < 5)
        {
    		usleep(100000); 	//100ms

            infrared_size = read(fd, buf, 255);

            if (infrared_size > 1)
            {
                break;
            }
            count ++;
        }

		if (infrared_size > 1)
        {
			memcpy(read_buf, buf+10, 4);
            memcpy(cont_buf, buf, 1);
            ir_distance = atoi((const char*)read_buf)*0.01;

			infrared_value = 1;
        }
        else
        {
        	ir_distance = 0;
            infrared_value = 0;
        }

		printf("infrared_value=%d\n", infrared_value);
    }
    // Check End

    // IMPORTANT update the current state. This should be done once per control loop
    int update_ret = sn_update_data();

    if(update_ret!=0)
    {
      printf("Detected likely failure in SN. Ensure it is running.\n");
    }
    else
    {
      // Print some information about the different commands
      // This will only be done once since it is fixed
      /*
      static unsigned int printed_command_info = 0;
      if (printed_command_info == 0)
      {
        const char* cmd_name[SN_RC_NUM_CMD_TYPES];
        SnRcCommandType rc_cmd_type[SN_RC_NUM_CMD_TYPES]
          = { SN_RC_RATES_CMD, SN_RC_THRUST_ANGLE_CMD, SN_RC_ALT_HOLD_CMD,
            SN_RC_GPS_POS_HOLD_CMD, SN_RC_OPTIC_FLOW_POS_HOLD_CMD,
            SN_RC_VIO_POS_HOLD_CMD };

        for (ii = 0; ii < SN_RC_NUM_CMD_TYPES; ++ii)
        {
          cmd_name[ii] = sn_get_cmd_name(rc_cmd_type[ii]);
          printf("%s\n",cmd_name[ii]);

          const char* units[4];
          float min[4];
          float max[4];
          unsigned int jj = 0;
          for (jj = 0; jj < 4; ++jj)
          {
            units[jj] = sn_get_dimensioned_units(rc_cmd_type[ii], jj);
            min[jj] = sn_get_min_value(rc_cmd_type[ii], jj);
            max[jj] = sn_get_max_value(rc_cmd_type[ii], jj);
            printf("cmd%d has range [%f %s, %f %s]\n", jj, min[jj], units[jj],
                max[jj], units[jj]);
          }
        }

        printf("Live view starting in ");
        for (ii = 10; ii > 0; --ii)
        {
          printf("%d...",ii);
          fflush(stdout);
          usleep(1000000);
        }
        printf("\n");

        printed_command_info = 1;
      }

      printf("------------------------------------\n");
      // Read in current mode
      SnMode mode = (SnMode) snav_data->general_status.current_mode;
      if (mode == SN_SENSOR_ERROR_MODE)
        printf("mode = SN_SENSOR_ERROR_MODE\n");
      else if (mode == SN_WAITING_FOR_DEVICE_TO_CONNECT)
        printf("mode = SN_WAITING_FOR_DEVICE_TO_CONNECT\n");
      else if (mode == SN_EMERGENCY_KILL_MODE)
        printf("mode = SN_EMERGENCY_KILL_MODE\n");
      else if (mode == SN_EMERGENCY_LANDING_MODE)
        printf("mode = SN_EMERGENCY_LANDING_MODE\n");
      else if (mode == SN_THERMAL_IMU_CALIBRATION_MODE)
        printf("mode = SN_THERMAL_IMU_CALIBRATION_MODE\n");
      else if (mode == SN_STATIC_ACCEL_CALIBRATION_MODE)
        printf("mode = SN_STATIC_ACCEL_CALIBRATION_MODE\n");
      else if (mode == SN_OPTIC_FLOW_CAM_YAW_CALIBRATION_MODE)
        printf("mode = SN_OPTIC_FLOW_CAM_YAW_CALIBRATION_MODE\n");
      else if (mode == SN_MAGNETOMETER_CALIBRATION_MODE)
        printf("mode = SN_MAGNETOMETER_CALIBRATION_MODE\n");
      else if (mode == SN_CALIBRATION_SUCCESS)
        printf("mode = SN_CALIBRATION_SUCCESS\n");
      else if (mode == SN_CALIBRATION_FAILURE)
        printf("mode = SN_CALIBRATION_FAILURE\n");
      else if (mode == SN_ESC_RPM_MODE)
        printf("mode = SN_ESC_RPM_MODE\n");
      else if (mode == SN_ESC_PWM_MODE)
        printf("mode = SN_ESC_PWM_MODE\n");
      else if (mode == SN_RATE_MODE)
        printf("mode = SN_RATE_MODE\n");
      else if (mode == SN_THRUST_ANGLE_MODE)
        printf("mode = SN_THRUST_ANGLE_MODE\n");
      else if (mode == SN_ALT_HOLD_MODE)
        printf("mode = SN_ALT_HOLD_MODE\n");
      else if (mode == SN_THRUST_GPS_HOVER_MODE)
        printf("mode = SN_THRUST_GPS_HOVER_MODE\n");
      else if (mode == SN_GPS_POS_HOLD_MODE)
        printf("mode = SN_GPS_POS_HOLD_MODE\n");
      else if (mode == SN_OPTIC_FLOW_POS_HOLD_MODE)
        printf("mode = SN_OPTIC_FLOW_POS_HOLD_MODE\n");
      else if (mode == SN_VIO_POS_HOLD_MODE)
        printf("mode = SN_VIO_POS_HOLD_MODE\n");
      else
        printf("mode = SN_UNDEFINED_MODE\n");

      // Read in current state of propellers
      SnPropsState props_state = (SnPropsState) snav_data->general_status.props_state;
      if (props_state == SN_PROPS_STATE_NOT_SPINNING)
        printf("props_state = SN_PROPS_STATE_NOT_SPINNING\n");
      else if (props_state == SN_PROPS_STATE_STARTING)
        printf("props_state = SN_PROPS_STATE_STARTING\n");
      else if (props_state == SN_PROPS_STATE_SPINNING)
        printf("props_state = SN_PROPS_STATE_SPINNING\n");
      else
        printf("props_state = SN_PROPS_STATE_UNKNOWN\n");

      // Get the on ground flag
      if (snav_data->general_status.on_ground == 0)
        printf("flight control thinks vehicle is NOT ON GROUND\n");
      else
        printf("flight control thinks vehicle is ON GROUND\n");

      // Get the battery voltage
      printf("电压 voltage = %f\n", snav_data->general_status.voltage);
*/
      // Get the snav version
      char library_version[18];
      memset(library_version, 0, 18);
      memcpy(library_version, snav_data->version_info.library_version, 18);
      // Get the status of the IMU
      SnDataStatus imu_status = (SnDataStatus) snav_data->data_status.imu_0_status;
      // Get the baro status
      SnDataStatus baro_status = (SnDataStatus) snav_data->data_status.baro_0_status;
      // Get the RC status
      SnDataStatus rc_status = (SnDataStatus) snav_data->data_status.spektrum_rc_0_status;
       // Get the mag status
      SnDataStatus mag_status = (SnDataStatus) snav_data->data_status.mag_0_status;
      // Get the GPS status
      SnDataStatus gps_status = (SnDataStatus) snav_data->data_status.gps_0_status;
      // Get the sonar status
      SnDataStatus sonar_status = (SnDataStatus) snav_data->data_status.sonar_0_status;
       // Get the optic flow status
      SnDataStatus optic_flow_status = (SnDataStatus) snav_data->data_status.optic_flow_0_status;
      // Get the esc status
      SnDataStatus esc_feedback_status = (SnDataStatus) snav_data->data_status.esc_feedback_status;
      // Get static accel calib status
      SnCalibStatus static_accel_calib_status;
      sn_get_static_accel_calibration_status(&static_accel_calib_status);
      // Get dynamic accel calib status
      SnCalibStatus dynamic_accel_calib_status;
      sn_get_dynamic_accel_calibration_status(&dynamic_accel_calib_status);
      // Get thermal imu calib status
      SnCalibStatus thermal_imu_calib_status;
      sn_get_imu_thermal_calibration_status(&thermal_imu_calib_status);
      // Get optic flow cam yaw calib status
      SnCalibStatus optic_flow_cam_yaw_calib_status;
      sn_get_optic_flow_camera_yaw_calibration_status(&optic_flow_cam_yaw_calib_status);
       // Get magnetometer calib status
      SnCalibStatus mag_calib_status;
      sn_get_magnetometer_calibration_status(&mag_calib_status);

      //check current version   print $2,$3
    char get_wifi_ssid_cmd[128] = "cat /etc/hostapd.conf | grep ssid= | head -n 1 | cut -c6-";
    char wifi_ssid[64];

    FILE *fp = popen(get_wifi_ssid_cmd, "r");
    fgets(wifi_ssid, sizeof(wifi_ssid), fp);
    pclose(fp);

    if (wifi_ssid[strlen(wifi_ssid)-1] == '\n')
    {
        wifi_ssid[strlen(wifi_ssid)-1] = '\0';
    }

    printf("\nWifi-Name     =       %s\n", wifi_ssid);

    printf("------------------------------------------------------------\n");
    printf("Snav Version    =   %s\n", library_version);

    printf("\n------------------------------------------------------------\n");
    printf("IMU:                %d\n", imu_switch);
    printf("BARO:               %d\n", baro_switch);
    printf("MAG:                %d\n", mag_switch);
    printf("GPS:                %d\n", gps_switch);
    printf("SONAR:              %d\n", sonar_switch);
    printf("OPTIC_FLOW:         %d\n", optic_flow_switch);
    printf("ESC:                %d\n", esc_switch);
    printf("CAM:                %d\n", cam_switch);
    printf("INFRARED:           %d\n", infrared_switch);
    printf("------------------------------------------------------------\n");

    if (imu_switch == 1)
    {
        printf("------------------------------------------------------------\n");
        printf("1: imu_status   = %d -----------", imu_status);

        if (imu_status == SN_DATA_VALID)
        {
            if ((snav_data->imu_0_raw.lin_acc[0] >= -0.05f) && (snav_data->imu_0_raw.lin_acc[0] <= 0.05f)
                && (snav_data->imu_0_raw.lin_acc[1] >= -0.05f) && (snav_data->imu_0_raw.lin_acc[1] <= 0.05f)
                && (snav_data->imu_0_raw.lin_acc[2] >= 0.95f) && (snav_data->imu_0_raw.lin_acc[2] <= 1.05f)
                &&(snav_data->imu_0_raw.ang_vel[0] >= -0.05f) && (snav_data->imu_0_raw.ang_vel[0] <= 0.05f)
                && (snav_data->imu_0_raw.ang_vel[1] >= -0.05f) && (snav_data->imu_0_raw.ang_vel[1] <= 0.05f)
                && (snav_data->imu_0_raw.ang_vel[2] >= -0.05f) && (snav_data->imu_0_raw.ang_vel[2] <= 0.05f))
            {
                printf("Pass\n");
            }
            else
            {
                pass_false_flag = 1;
                printf("Fail003\n");
            }
        }
        else if (imu_status == SN_DATA_NOT_INITIALIZED)
        {
            pass_false_flag = 1;
            printf("Fail001\n");
        }
        else if (imu_status == SN_DATA_UNCALIBRATED)
        {
            pass_false_flag = 1;
            printf("Fail002\n");
        }
        else
        {
            pass_false_flag = 1;
            printf("Fail004\n");
        }
    }

    if (baro_switch == 1)
    {
        printf("------------------------------------------------------------\n");
        printf("2: baro_status = %d -----------", baro_status);
        //print_sn_data_status_string(baro_status);
        if (baro_status == SN_DATA_VALID)
        {
            printf("Pass\n");
        }
        else
        {
            pass_false_flag = 1;
            printf("Fail\n");
        }
    }

      /*
    printf("------------------------------------------------------------\n");
      printf("rc_status         = ");
      //print_sn_data_status_string(rc_status);
      if (rc_status == SN_DATA_VALID)
        printf("Pass\n");
      else
        {
          pass_false_flag = 1;
          printf("Fail\n");
      }*/

    if (mag_switch == 1)
    {
        printf("------------------------------------------------------------\n");
        printf("3: mag_status = %d -----------", mag_status);
        //print_sn_data_status_string(mag_status);
        if (mag_status == SN_DATA_VALID)
        {
            printf("Pass\n");
        }
        else if (mag_status == SN_DATA_WARNING)
        {
            printf("Pass-W\n");
        }
        else
        {
            pass_false_flag = 1;
            printf("Fail\n");
        }
    }


    if (gps_switch == 1)
    {
        printf("------------------------------------------------------------\n");
        printf("4: gps_status = %d -----------", gps_status);
        //print_sn_data_status_string(gps_status);
        if ((gps_status == SN_DATA_VALID) || (gps_status == SN_DATA_NO_LOCK ))
        {
            printf("Pass\n");
        }
        else
        {
            pass_false_flag = 1;
            printf("Fail\n");
        }
    }


    if (sonar_switch == 1)
    {
        printf("------------------------------------------------------------\n");
        printf("5: sonar_status = %d -----------", sonar_status);
        //print_sn_data_status_string(sonar_status);
        if (sonar_status == SN_DATA_VALID)
        {
            printf("Pass\n");
        }
        else
        {
            pass_false_flag = 1;
            printf("Fail\n");
        }
    }

    if (optic_flow_switch == 1)
    {
        printf("------------------------------------------------------------\n");
        printf("6: optic_flow_status = %d -----------", optic_flow_status);
        //print_sn_data_status_string(optic_flow_status);
        if (optic_flow_status == SN_DATA_VALID)
        {
            printf("Pass\n");
        }
        else if (optic_flow_status == SN_DATA_WARNING)
        {
            printf("Pass-W\n");
        }
        else
        {
            pass_false_flag = 1;
            printf("Fail\n");
        }
    }

    if (esc_switch == 1)
    {
        printf("------------------------------------------------------------\n");
        printf("7: esc_feedback_status = %d -----------", esc_feedback_status);
        if (esc_feedback_status == SN_DATA_VALID)
        {
            if ((snav_data->esc_raw.voltage[0]>=6.0f && snav_data->esc_raw.voltage[0]<=8.5f)
                && (snav_data->esc_raw.voltage[1]>=6.0f && snav_data->esc_raw.voltage[1]<=8.5f)
                && (snav_data->esc_raw.voltage[2]>=6.0f && snav_data->esc_raw.voltage[2]<=8.5f)
                && (snav_data->esc_raw.voltage[3]>=6.0f && snav_data->esc_raw.voltage[3]<=8.5f))
            {
                printf("Pass\n");
            }
            else
            {
                pass_false_flag = 1;
                printf("Fail002\n");
            }
        }
        else
        {
            pass_false_flag = 1;
            printf("Fail001\n");
        }
    }

    if (cam_switch == 1)
    {
        printf("------------------------------------------------------------\n");
        printf("8: cam_status = %d -----------", cam_value);
        if (cam_value == 1)
        {
            printf("Pass\n");
        }
        else
        {
            pass_false_flag = 1;
            printf("Fail\n");
        }
    }

    if (infrared_switch == 1)
    {
        printf("------------------------------------------------------------\n");
        printf("9: infrared_data = %f -----------", ir_distance);
        if (infrared_value == 1)
        {
            printf("Pass\n");
        }
        else
        {
            pass_false_flag = 1;
            printf("Fail\n");
        }
    }
      /*
      printf("-----------------------------------------------------------------------------------------\n");
      printf("静态校准       static_accel_calib_status       = ");
      //print_sn_calib_status_string(static_accel_calib_status);
      if (static_accel_calib_status == SN_CALIB_STATUS_CALIBRATED)
        printf("已校准\n");
      else
        {
          pass_false_flag = 1;
          printf("FALSE\n");
      }
    printf("-----------------------------------------------------------------------------------------\n");
      printf("动态校准       dynamic_accel_calib_status      = ");
      //print_sn_calib_status_string(dynamic_accel_calib_status);
      if (dynamic_accel_calib_status == SN_CALIB_STATUS_CALIBRATED)
        printf("已校准\n");
      else
        {
          pass_false_flag = 1;
          printf("FALSE\n");
      }
    printf("-----------------------------------------------------------------------------------------\n");
      printf("陀螺仪温度校准 thermal_imu_calib_status        = ");
      //print_sn_calib_status_string(thermal_imu_calib_status);
      if (thermal_imu_calib_status == SN_CALIB_STATUS_CALIBRATED)
        printf("已校准\n");
      else
        {
          pass_false_flag = 1;
          printf("FALSE\n");
      }
    printf("-----------------------------------------------------------------------------------------\n");
      printf("光流YAW校准    optic_flow_cam_yaw_calib_status = ");
      //print_sn_calib_status_string(optic_flow_cam_yaw_calib_status);
      if (optic_flow_cam_yaw_calib_status == SN_CALIB_STATUS_CALIBRATED)
        printf("已校准\n");
      else
        {
          pass_false_flag = 1;
          printf("FALSE\n");
      }
    printf("-----------------------------------------------------------------------------------------\n");
      printf("地磁校准       mag_calib_status                = ");
      //print_sn_calib_status_string(mag_calib_status);
      if (mag_calib_status == SN_CALIB_STATUS_CALIBRATED)
        printf("已校准\n");
      else
        {
          pass_false_flag = 1;
          printf("FALSE\n");
      }
      */

    printf("\n\n\n");

    if(pass_false_flag == 1)
    {
        printf("     #########      #        #   #          \n");
        printf("     #            #   #      #   #          \n");
        printf("     #########   #######     #   #          \n");
        printf("     #          #       #    #   #          \n");
        printf("     #         #         #   #   ########## \n");

    }
    else
    {
        printf("     #########     #       #########   ######### \n");
        printf("     #       #   #   #     #           #         \n");
        printf("     #########  #######    #########   ######### \n");
        printf("     #         #       #           #           # \n");
        printf("     #        #         #  #########   ######### \n");
    }

        }

    //Sleep until next loop
    usleep(100000);
    }

	close(fd);
  return 0;
}
