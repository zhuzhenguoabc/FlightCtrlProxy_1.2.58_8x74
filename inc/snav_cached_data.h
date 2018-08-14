/*****************************************************************************
 *
 * Copyright 2015-2017 Qualcomm Technologies, Inc.  All rights reserved.
 *
 * This software may be subject to U.S. and international export, re-export
 * or transfer laws. Diversion contrary to U.S. and international law is
 * strictly prohibited.
 *
 * The party receiving this software directly from QTI (the "Recipient")
 * may use this software solely as set forth in the agreement between the
 * Recipient and QTI (the "Agreement"). The software may be used in source
 * code form solely by the Recipient's employees (if any) authorized by the
 * Agreement. Unless expressly authorized in the Agreement, the Recipient
 * may not sublicense, assign, transfer or otherwise provide the source
 * code to any third party. Qualcomm Technologies, Inc. retains all
 * ownership rights in and to the software. Except as may be expressly
 * granted by the Agreement, this file provides no license to any patents,
 * trademarks, copyrights, or other intellectual property of QUALCOMM
 * Incorporated or its affiliates.
 *
 * This notice supersedes any other QTI notices contained within the
 * software except copyright notices indicating different years of
 * publication for different portions of the software. This notice does not
 * supersede the application of any third party copyright notice to that
 * third party's code.
 ****************************************************************************/
#ifndef SNAV_CACHED_DATA_H_
#define SNAV_CACHED_DATA_H_

#include <stdint.h>

#include "snav_types.h"

/** @addtogroup sn_datatypes
@{ */
/**
 * Version information required to uniquely identify the software and device.
 */
typedef struct
{
  char device_identifier[20];
  /**< Version of the DSPaL library used. */
  char compile_date[16];
  /**< Null terminated string containing the compilation date. */
  char compile_time[16];
  /**< Null terminated string containing the compilation time. */
  char library_version[18];
  /**< Null terminated string representing version information. */
  char library_hash[41];
  /**< Null terminated string with a unique build identifier. */
  char mac_address[18];
  /**< Null terminated string containing wlan0 mac address if it was successfully polled. */
  char dspal_version[40];
  /**< Version of the DSPaL library used. */
  int32_t esc_hw_version[8];
  /**< Hardware revision of the ESCs. */
  int32_t esc_sw_version[8];
  /**< Software version of the ESCs. */
} VersionInfo;

/**
 * Machine vision (MV) version information.
 */
typedef struct
{
  char version_recommended[18];
  /**< Null-terminated string containing the recommended MV SDK version. */
  char version_found[18];
  /**< Null-terminated string containing the MV SDK version found on the system. */
  uint8_t strict_checking;
  /**< Specifies if the recommended MV SDK version is required. @newpagetable */
} MvSdkVersionInfo;

/**
 * Version of the sensor_imu API.
 */
typedef struct
{
  char version_recommended[18];
  /**< Null-terminated string containing the recommended sensor_imu API version. */
  char version_found[18];
  /**< Null-terminated string containing the sensor_imu API version found on the system. */
  uint8_t strict_checking;
  /**< Specifies if the recommended sensor_imu API version is required. */
} SensorImuApiVersionInfo;

/**
 * General system state information for debugging system issues.
 */
typedef struct
{
  int64_t time;
  /**< Time at which the most recent iteration of the main flight loop started.  (Units: @us) */
  uint32_t loop_cntr;
  /**< Number of times the control loop has run. */
  int32_t desired_mode;
  /**< Cast to enum: #SnMode. Desired mode that the flight controller attempts to transition to. */
  int32_t current_mode;
  /**< Cast to enum: #SnMode. Current flight controller mode. */
  float voltage;
  /**< Estimated input system voltage.  (Units: V) */
  float current;
  /**< If available, the estimated electrical current being used by the system.  (Units: A) */
  float charge_used;
  /**< Charge used by the vehicle since initialization of the flight controller.  (Units: mAh) */
  uint8_t is_using_external_voltage;
  /**< 1 -- Voltage is being measured by the external voltage driver.  0 -- Voltage is measured using ESCs. */
  int32_t props_state;
  /**< Cast to enum: #SnPropsState. Propeller state. */
  uint8_t on_ground;
  /**< Flag representing flight controller's detection of the device being on the ground. 1 -- Device on the ground, 0 otherwise. */
  int32_t input_cmd_type;
  /**< Cast to enum: #SnInputCommandType. Input command type. */
  char last_error_code[32];
  /**< Null-terminated string to represent the last error code detected. To allow detection of infrequent errors, this string persists even if cleared by an error code. */
} GeneralStatus;

/**
 * Status of various sensors and estimators.
 */
typedef struct
{
  int64_t time;
  /**< Time at which the struct was logged. (Units: ms) */
  uint32_t loop_cntr;
  /**< Number of times at which the control loop has run. */
  int32_t imu_0_status;
  /**< Cast to enum: #SnDataStatus. IMU0 sensor status. */
  int32_t imu_1_status;
  /**< Cast to enum: #SnDataStatus. IMU1 sensor status. */
  int32_t imu_2_status;
  /**< Cast to enum: #SnDataStatus. IMU2 sensor status. */
  int32_t baro_0_status;
  /**< Cast to enum: #SnDataStatus. Barometer sensor status. */
  int32_t esc_feedback_status;
  /**< Cast to enum: #SnDataStatus. ESC feedback status. */
  int32_t mag_0_status;
  /**< Cast to enum: #SnDataStatus. Magnetometer sensor. */
  int32_t gps_0_status;
  /**< Cast to enum: #SnDataStatus. Global positioning GPS sensor status. */
  int32_t sonar_0_status;
  /**< Cast to enum: #SnDataStatus. Sonar sensor status. */
  int32_t optic_flow_0_status;
  /**< Cast to enum: #SnDataStatus. Optic flow (DFT) sensor status. */
  int32_t spektrum_rc_0_status;
  /**< Cast to enum: #SnDataStatus. Spektrum RC sensor status. */
  int32_t api_rc_status;
  /**< Cast to enum: #SnDataStatus. API RC command status. */
  int32_t rc_active_status;
  /**< Cast to enum: #SnDataStatus. Active RC command status. */
  int32_t height_estimator_status;
  /**< Cast to enum: #SnDataStatus. Height estimator status. */
  int32_t attitude_estimator_status;
  /**< Cast to enum: #SnDataStatus. Attitude estimator status. */
  int32_t vio_0_status;
  /**< Cast to enum: #SnDataStatus. Visual inertial odometry (VIO) status. */
  int32_t voa_status;
  /**< Cast to enum: #SnDataStatus. Visual obstacle avoidance (VOA) status. */
  int32_t api_trajectory_status;
  /**< Cast to enum: #SnDataStatus. Status of trajectory data. */
} DataStatus;

/**
 * System and sensor update rates.
 */
typedef struct
{
  int64_t time;
  /**< Time at which the struct was logged. (Units: @us) */
  uint32_t loop_cntr;
  /**< Number of times the control loop has run. */
  float control_loop_freq;
  /**< Main control loop update frequency.  (Units: Hz) */
  float imu0_freq;
  /**< IMU0 update frequency.  (Units: Hz) */
  float imu1_freq;
  /**< IMU1 update frequency.  (Units: Hz) */
  float imu2_freq;
  /**< IMU2 update frequency.  (Units: Hz) */
  float baro0_freq;
  /**< Baro0 update frequency.  (Units: Hz) */
  float mag0_freq;
  /**< Mag0 update frequency.  (Units: Hz) */
  float sonar0_freq;
  /**< Sonar0 update frequency.  (Units: Hz) */
  float rc0_freq;
  /**< RC0 update frequency.  (Units: Hz) */
  float esc_fb_freq;
  /**< ESC feedback update frequency.  (Units: Hz) */
  float gnss0_freq;
  /**< GNSS0 update frequency.  (Units: Hz) */
  float gnss1_freq;
  /**< GNSS1 update frequency.  (Units: Hz) */
  float obstacle_freq;
  /**< Obstacle update frequency.  (Units: Hz) */
  float vio0_freq;
  /**< VIO0 update frequency.  (Units: Hz) */
} UpdateRates;

/**
 * Estimate of the vehicle orientation.
 */
typedef struct
{
  int64_t time;
  /**< Timestamp.  (Units: @us) */
  uint32_t cntr;
  /**< Counter incremented upon successful computation of the attitude estimate. */
  float roll;
  /**< Roll angle using Tait-Bryan ZYX.  (Units: rad) */
  float pitch;
  /**< Pitch angle using Tait-Bryan ZYX.  (Units: rad) */
  float yaw;
  /**< Yaw angle using Tait-Bryan ZYX.  (Units: rad) */
  float rotation_matrix[9];
  /**< Rotation matrix from the vehicle body to world in row-major order. */
  float magnetic_yaw_offset;
  /**< Yaw angle with respect to magnetic east = yaw + magnetic_yaw_offset. (Units: rad) */
  float magnetic_declination;
  /**< Yaw angle with respect to true east = yaw + magnetic_yaw_offset + magnetic_declination. (Units: rad) */
} AttitudeEstimate;

/**
 * Estimate of the vehicle orientation.
 */
typedef struct
{
  int64_t time;
  /**< Timestamp.  (Units: @us) */
  uint32_t cntr;
  /**< Counter incremented upon successful computation of the attitude estimate. */
  float roll;
  /**< Roll angle using Tait-Bryan ZYX.  (Units: rad) */
  float pitch;
  /**< Pitch angle using Tait-Bryan ZYX.  (Units: rad) */
  float yaw;
  /**< Yaw angle using Tait-Bryan ZYX.  (Units: rad) */
  float rotation_matrix[9];
  /**< Rotation matrix from the vehicle body to world in row-major order. */
  float magnetic_yaw_offset;
  /**< Yaw angle with respect to magnetic east = yaw + magnetic_yaw_offset. (Units: rad) */
  float magnetic_declination;
  /**< Yaw angle with respect to true east = yaw + magnetic_yaw_offset + magnetic_declination. (Units: rad) */
} AttitudeEstimate1;

/**
 * Estimate of the vehicle orientation.
 */
typedef struct
{
  int64_t time;
  /**< Timestamp.  (Units: @us) */
  uint32_t cntr;
  /**< Counter incremented upon successful computation of the attitude estimate. */
  float roll;
  /**< Roll angle using Tait-Bryan ZYX.  (Units: rad) */
  float pitch;
  /**< Pitch angle using Tait-Bryan ZYX.  (Units: rad) */
  float yaw;
  /**< Yaw angle using Tait-Bryan ZYX.  (Units: rad) */
  float rotation_matrix[9];
  /**< Rotation matrix from the vehicle body to world in row-major order. */
  float magnetic_yaw_offset;
  /**< Yaw angle with respect to magnetic east = yaw + magnetic_yaw_offset. (Units: rad) */
  float magnetic_declination;
  /**< Yaw angle with respect to true east = yaw + magnetic_yaw_offset + magnetic_declination. (Units: rad) */
} AttitudeEstimate2;

/**
 * Apps processor CPU status.
 */
typedef struct
{
  int64_t time;
  /**< Time at which the data was logged. (Units: @us) */
  uint32_t cntr;
  /**< Number of times data was updated */
  uint64_t time_apps_us;
  /**< Timestamp from the Apps processor.  (Units: @us) */
  uint64_t time_apps_real_us;
  /**< Timestamp from the Apps processor realtime clock.  (Units: @us) */
  float cur_freq[4];
  /**< Current Apps processor CPU frequency. If the CPU is not online, Snapdragon Navigator reads NaN.  (Units: GHz) */
  float max_freq[4];
  /**< Maximum Apps processor CPU frequency -- Can be throttled due to temperature. If the CPU is not online, Snapdragon Navigator reads NaN.  (Units: GHz) */
  float temp[22];
  /**< Temperature measurements from thermal zones. Reads NaN if the thermal zone is disabled.  (Units: @degc) */
} CpuStats;

/**
 * Inertial measurement unit 0 raw data.
 */
typedef struct
{
  uint32_t iter;
  /**< Loop iteration in which data was logged. */
  int64_t time;
  /**< Time at which the data was received.  (Units: @us) */
  uint32_t cntr;
  /**< Number of measurements received. */
  float temp;
  /**< IMU temperature.  (Units: @degc) */
  float lin_acc[3];
  /**< Linear acceleration.  (Units: gravity (~9.81 m/s/s)) */
  float ang_vel[3];
  /**< Angular velocity.  (Units: rad/s) */
} Imu0Raw;

/**
 * Inertial measurement unit 1 raw data.
 */
typedef struct
{
  uint32_t iter;
  /**< Loop iteration in which data was logged. */
  int64_t time;
  /**< Time at which the data was received.  (Units: @us) */
  uint32_t cntr;
  /**< Number of measurements received. */
  float temp;
  /**< IMU temperature.  (Units: @degc) */
  float lin_acc[3];
  /**< Linear acceleration.  (Units: gravity (~9.81 m/s/s)) */
  float ang_vel[3];
  /**< Angular velocity.  (Units: rad/s) */
} Imu1Raw;

/**
 * Inertial measurement unit 2 raw data.
 */
typedef struct
{
  uint32_t iter;
  /**< Loop iteration in which data was logged. */
  int64_t time;
  /**< Time at which the data was received.  (Units: @us) */
  uint32_t cntr;
  /**< Number of measurements received. */
  float temp;
  /**< IMU temperature.  (Units: @degc) */
  float lin_acc[3];
  /**< Linear acceleration.  (Units: gravity (~9.81 m/s/s)) */
  float ang_vel[3];
  /**< Angular velocity.  (Units: rad/s) */
} Imu2Raw;

/**
 * Inertial measurement unit (IMU) data after compensation.
 */
typedef struct
{
  int64_t time;
  /**< Time data was received.  (Units: @us) */
  uint32_t cntr;
  /**< Number of compensated measurements recorded. */
  float temp;
  /**< Temperature of IMU.  (Units: @degc) */
  float lin_acc[3];
  /**< Linear acceleration.  (Units: gravity (~9.81 m/s/s)) */
  float ang_vel[3];
  /**< Angular velocity.  (Units: rad/s) */
} Imu0Compensated;

/**
 * Inertial measurement unit (IMU) data after compensation.
 */
typedef struct
{
  int64_t time;
  /**< Time data was received.  (Units: @us) */
  uint32_t cntr;
  /**< Number of compensated measurements recorded. */
  float temp;
  /**< Temperature of IMU.  (Units: @degc) */
  float lin_acc[3];
  /**< Linear acceleration.  (Units: gravity (~9.81 m/s/s)) */
  float ang_vel[3];
  /**< Angular velocity.  (Units: rad/s) */
} Imu1Compensated;

/**
 * Inertial measurement unit (IMU) data after compensation.
 */
typedef struct
{
  int64_t time;
  /**< Time data was received.  (Units: @us) */
  uint32_t cntr;
  /**< Number of compensated measurements recorded. */
  float temp;
  /**< Temperature of IMU.  (Units: @degc) */
  float lin_acc[3];
  /**< Linear acceleration.  (Units: gravity (~9.81 m/s/s)) */
  float ang_vel[3];
  /**< Angular velocity.  (Units: rad/s) */
} Imu2Compensated;

/**
 * Data results from IMU temperature calibration.
 */
typedef struct
{
  float accel_slope[3];
  /**< XYZ slopes for accelerometer temperature calibration. \n  (Units: gravity/@degc) */
  float accel_offset[3];
  /**< XYZ offsets for accelerometer temperature calibration. \n  (Units: gravity) */
  float accel_residual[3];
  /**< Average squared residual for accelerometer temperature calibration.  (Units: gravity^2) */
  float gyro_slope[3];
  /**< XYZ slopes for gyroscope temperature calibration. \n  (Units: (rad/s)/@degc) */
  float gyro_offset[3];
  /**< XYZ offsets for gyroscope temperature calibration.  (Units: (rad/s))) */
  float gyro_residual[3];
  /**< Average squared residual for gyroscope temperature calibration.  (Units: (rad/s)^2) */
} Imu0CalibrationThermal;

/**
 * IMU sensor offset values.
 */
typedef struct
{
  char name[16];
  /**< Type of offset calibration (e.g., static or dynamic). */
  float accel_offset[3];
  /**< XYZ accelerometer offsets from accelerometer offset calibration.  (Units: gravity (~9.81 m/s/s)) */
  float avg_thrust;
  /**< Average thrust found from in flight accelerometer calibration.  (Units: g) */
  float roll_trim_offset;
  /**< Roll trim offset from in flight accelerometer calibration. \n  (Units: g) */
  float pitch_trim_offset;
  /**< Pitch trim offset from in flight accelerometer calibration.  (Units: g) */
} Imu0CalibrationOffset;

/**
 * Raw barometer data.
 */
typedef struct
{
  uint32_t iter;
  /**< Loop iteration in which data was logged. */
  int64_t time;
  /**< Time at which the struct was logged. (Units: @us) */
  uint32_t cntr;
  /**< Number of times data was read. */
  float pressure;
  /**< Atmospheric pressure measurement.  (Units: Pa) */
  float temp;
  /**< Temperature of the sensor.  (Units: @degc) */
} Barometer0Raw;

/**
 * Raw sonar data.
 */
typedef struct
{
  uint32_t iter;
  /**< Loop iteration in which data was logged. */
  int64_t time;
  /**< Time at which the struct was logged. (Units: @us) */
  uint32_t cntr;
  /**< Number of times the data was read. */
  float range;
  /**< Range measurement.  (Units: m) */
} Sonar0Raw;

/**
 * Raw magnetometer data from the mag0 sensor.
 */
typedef struct
{
  uint32_t iter;
  /**< Loop iteration in which data was logged. */
  int64_t time;
  /**< Time at which the data was received.  (Units: @us) */
  uint32_t cntr;
  /**< Number of data packets received. */
  uint8_t identifier;
  /**< Type of compass sensor. */
  float field[3];
  /**< XYZ components of the magnetic field in the sensor frame.  (Units: @uT) */
} Mag0Raw;

/**
 * Raw magnetometer data from the mag1 sensor.
 */
typedef struct
{
  uint32_t iter;
  /**< Loop iteration in which data was logged. */
  int64_t time;
  /**< Time at which the data was received.  (Units: @us) */
  uint32_t cntr;
  /**< Number of data packets received. */
  uint8_t identifier;
  /**< Type of compass sensor. */
  float field[3];
  /**< XYZ components of the magnetic field in the sensor frame. (Units: @uT) */
} Mag1Raw;

/**
 * Mag0 sensor data after compensation.
 */
typedef struct
{
  int64_t time;
  /**< Time at which the data was received.  (Units: @us) */
  uint32_t cntr;
  /**< Number of data packets received. */
  uint8_t identifier;
  /**< Type of compass sensor. */
  float field[3];
  /**< XYZ components of the magnetic field in the sensor frame. */
} Mag0Compensated;

/**
 * Mag1 sensor data after compensation.
 */
typedef struct
{
  int64_t time;
  /**< Time at which the data was received.  (Units: @us) */
  uint32_t cntr;
  /**< Number of data packets received. */
  uint8_t identifier;
  /**< Type of compass sensor. */
  float field[3];
  /**< XYZ components of the magnetic field in the sensor frame. */
} Mag1Compensated;

/**
 * Data results from Mag0 3D calibration.
 */
typedef struct
{
  float matrix[9];
  /**< Scale parameters of mapping. */
  float offset[3];
  /**< XYZ offset of mapping. */
  float rot[9];
  /**< Mag Rotation on vehicle from calibration file. */
} Mag0Calibration3D;

/**
 * Raw Spektrum RC data.
 */
typedef struct
{
  uint32_t iter;
  /**< Loop iteration in which data was logged. */
  int64_t time;
  /**< Time at which the struct was logged. (Units: @us) */
  uint32_t cntr;
  /**< Number of times the data was read. */
  int32_t protocol;
  /**< Cast to enum: #SnRcReceiverMode. RC protocol identifier */
  uint8_t num_channels;
  /**< Number of RC channels being populated. */
  uint16_t vals[16];
  /**< Raw Spektrum channel values. */
} SpektrumRc0Raw;

/**
 * RC commands sent through the API.
 */
typedef struct
{
  uint32_t iter;
  /**< Loop iteration in which data was logged. */
  int64_t time;
  /**< Time at which the struct was logged. (Units: @us) */
  uint32_t cntr;
  /**< Number of times the data was read. */
  int32_t cmd_type;
  /**< Cast to enum: #SnRcCommandType. How the command is interpreted (if possible). */
  int32_t cmd_options;
  /**< Cast to enum: #SnRcCommandOptions. Options used to deviate from linear mapping. */
  float cmd[4];
  /**< Unitless RC-type command in range [-1, 1]. */
} ApiRcRaw;

/**
 * Thrust, attitude, and angular velocity commands sent through the API.
 */
typedef struct
{
  uint32_t iter;
  /**< Loop iteration in which data was logged. */
  float thrust;
  /**< Thrust. */
  float qw;
  /**< Attitude quaternion W value. */
  float qx;
  /**< Attitude quaternion X value. */
  float qy;
  /**< Attitude quaternion Y value. */
  float qz;
  /**< Attitude quaternion Z value. */
  float ang_vel[3];
  /**< Angular velocity. */
} ApiThrustAttAngVel;

/**
 * Received API spin or stop propellers commands.
 */
typedef struct
{
  uint32_t iter;
  /**< Loop iteration in which data was logged. */
  uint8_t api_spin_props_rcvd;
  /**< 1 if spin-propellers command received, 0 otherwise. */
  uint8_t api_stop_props_rcvd;
  /**< 1 if stop-propellers command received, 0 otherwise. */
} ApiPropsCmd;

/**
 * RC commands for control.
 */
typedef struct
{
  int64_t time;
  /**< Time at which the struct was logged. (Units: @us) */
  uint32_t cntr;
  /**< Number of times the data was read. */
  int32_t source;
  /**< Cast to enum: #SnRcCommandSource. Source of the active RC commands. */
  int32_t cmd_type;
  /**< Cast to enum: #SnRcCommandType. Specifies how the commands are interpreted (if possible). Irrelevant if the source is Spektrum RC. */
  int32_t cmd_options;
  /**< Cast to enum: #SnRcCommandOptions. Options used to deviate from linear mapping. */
  float cmd[4];
  /**< Unitless RC-type command in range [-1, 1]. */
} RcActive;

/**
 * Captured camera frame information from the downward camera.
 */
typedef struct
{
  int64_t time;
  /**< Time at which the struct was logged. (Units: @us) */
  uint32_t cntr;
  /**< Number of times the data was read. */
  uint32_t frame_num;
  /**< Frame number received, starting at 0. */
  int64_t frame_timestamp;
  /**< Timestamp of frame.  (Units: @us) */
  int64_t preview_timestamp;
  /**< Timestamp of preview callback.  (Units: @us) */
  float exposure;
  /**< Normalized exposure setting used to take the frame. */
  float gain;
  /**< Normalized gain setting used to take the frame. */
  float average_luminance;
  /**< Normalized average luminance of the frame. */
} Camera0FrameInfo;

/**
 * Downward facing tracker (DFT) data.
 */
typedef struct
{
  uint32_t iter;
  /**< Loop iteration in which data was logged. */
  int64_t time;
  /**< Time at which the struct was logged. (Units: @us) */
  uint32_t cntr;
  /**< Number of times the data was read. */
  float pixel_flow[2];
  /**< Pixel displacement between subsequent image frames. Pixel flow is in the opposite direction of camera (and therefore vehicle) movement.  (Units: pixels) */
  int32_t sample_size;
  /**< Number of inliers after calculation of displacement. */
  float error_sum;
  /**< Error metric, sum of squared error over sample size points. */
} OpticFlow0Raw;

/**
 * Downfacing camera calibration for tilt angle (optic flow camera yaw calibration).
 */
typedef struct
{
  float x_factor;
  /**< Tilt factor in X.  (Units: pixels/rad) */
  float y_factor;
  /**< Tilt factor in Y.  (Units: pixels/rad) */
} OpticFlow0CalibrationTilt;

/**
 * Raw GPS data.
 */
typedef struct
{
  uint32_t iter;
  /**< Loop iteration in which data was logged. */
  int64_t time;
  /**< Time at which the data was received.  (Units: @us) */
  uint32_t cntr;
  /**< Number of complete messages received. */
  int32_t identifier;
  /**< Cast to enum: #SnGnssReceiverType. Type of GNSS receiver. */
  uint32_t num_errors;
  /**< Number of CRC errors. */
  uint32_t gps_week;
  /**< GPS week number.  (Units: weeks) */
  uint32_t gps_time_sec;
  /**< Time of week.  (Units: sec) */
  uint32_t gps_time_nsec;
  /**< Time of week.  (Units: ns) */
  int32_t latitude;
  /**< Position latitude.  (Units: deg*10e7) */
  int32_t longitude;
  /**< Position longitude.  (Units: deg*10e7) */
  float altitude;
  /**< Altitude at mean sea level (MSL).  (Units: m) */
  float lin_vel[3];
  /**< Velocity of the east-north-up (ENU) frame.  (Units: m/s) */
  uint8_t fix_type;
  /**< Fix type/quality. */
  uint8_t num_satellites;
  /**< Number of satellites used in the solution. */
  float horizontal_acc;
  /**< Horizontal accuracy of the position estimate.  (Units: m) */
  float speed_acc;
  /**< Horizontal speed accuracy.  (Units: m/s) */
  float speed_up_acc;
  /**< Vertical speed accuracy.  (Units: m/s) */
  uint8_t sv_ids[32];
  /**< Satellite identification number. */
  uint8_t sv_cn0[32];
  /**< Satellite signal strength.  (Units: C/N0) */
} Gps0Raw;

/**
 * Raw GPS data.
 */
typedef struct
{
  uint32_t iter;
  /**< Loop iteration in which data was logged. */
  int64_t time;
  /**< Time at which the data was received.  (Units: @us) */
  uint32_t cntr;
  /**< Number of complete messages received. */
  int32_t identifier;
  /**< Cast to enum: #SnGnssReceiverType. Type of GNSS receiver. */
  uint32_t num_errors;
  /**< Number of CRC errors. */
  uint32_t gps_week;
  /**< GPS week number.  (Units: weeks) */
  uint32_t gps_time_sec;
  /**< Time of week.  (Units: sec) */
  uint32_t gps_time_nsec;
  /**< Time of week.  (Units: ns) */
  int32_t latitude;
  /**< Position latitude.  (Units: deg*10e7) */
  int32_t longitude;
  /**< Position longitude.  (Units: deg*10e7) */
  float altitude;
  /**< Altitude at mean sea level (MSL).  (Units: m) */
  float lin_vel[3];
  /**< Velocity of the east-north-up (ENU) frame.  (Units: m/s) */
  uint8_t fix_type;
  /**< Fix type/quality. */
  uint8_t num_satellites;
  /**< Number of satellites used in the solution. */
  float horizontal_acc;
  /**< Horizontal accuracy of the position estimate.  (Units: m) */
  float speed_acc;
  /**< Horizontal speed accuracy.  (Units: m/s) */
  float speed_up_acc;
  /**< Vertical speed accuracy.  (Units: m/s) */
  uint8_t sv_ids[32];
  /**< Satellite identification number. */
  uint8_t sv_cn0[32];
  /**< Satellite signal strength.  (Units: C/N0) */
} Gps1Raw;

/**
 * Raw API data for trajectory control input.
 */
typedef struct
{
  int64_t time;
  /**< Timestamp.  (Units: @us) */
  uint32_t cntr;
  /**< Number of messages received. */
  int32_t controller;
  /**< Cast to enum: #SnPositionController. Desired position controller to use. */
  int32_t options;
  /**< Cast to enum: #SnTrajectoryOptions. Trajectory options. */
  float position[3];
  /**< Desired position for the controller to achieve. (Units: m) */
  float velocity[3];
  /**< Desired velocity for the controller to achieve. (Units: m/s) */
  float acceleration[3];
  /**< Desired acceleration for controller to use as a feed-forward term. (Units: m/s/s) */
  float yaw;
  /**< Desired gravity aligned yaw angle for the controller to achieve. (Units: raw) */
  float yaw_rate;
  /**< Desired gravity aligned yaw angle rate for the controller to achieve. (Units: raw) */
} TrajectoryDataRaw;

/**
 * Position and velocity control data.
 */
typedef struct
{
  int64_t time;
  /**< Time at which the data was logged. (Units: @us) */
  uint32_t cntr;
  /**< Number of times the data was logged. */
  float position_estimated[3];
  /**< Estimated position of the vehicle with respect to the estimation frame. (Units: m) */
  float velocity_estimated[3];
  /**< Estimated velocity of the vehicle with respect to the estimation frame. (Units: m/s) */
  float yaw_estimated;
  /**< Estimated yaw angle of the vehicle with respect to the estimation frame. (Units: rad) */
  uint8_t estimate_is_valid;
  /**< Whether the estimate is valid. */
  int32_t position_estimate_type;
  /**< Cast to enum: #SnPosEstType. Names the dominant source of the position estimate. */
  float R_eg[9];
  /**< Orientation of the GNSS ENU frame with respect to the estimation frame. */
  uint8_t gnss_rotation_is_valid;
  /**< Indicates if the rotation between the estimation frame and the GNSS ENU frame is valid or not. */
  float t_eg[3];
  /**< Vector from the origin of the estimation frame to the origin of the GNSS ENU frame represented with respect to the estimation frame. If the estimation frame were drift-free, this vector would be zero over long time scales (could be non-zero over short time scales due to filtering delays). (Units: m) */
  uint8_t gnss_translation_is_valid;
  /**< Indicates if the translation between the estimation frame and the GNSS ENU frame is valid or not. */
  int32_t gnss_type;
  /**< Cast to enum: #SnGnssReceiverType. Type of GNSS receiver. */
  float position_desired[3];
  /**< Desired position of the vehicle with respect to the estimation frame. (Units: m) */
  float velocity_desired[3];
  /**< Desired velocity of the vehicle with respect to the estimation frame. (Units: m/s) */
  float yaw_desired;
  /**< Desired yaw of the vehicle with respect to the estimation frame. (Units: rad) */
  float t_el[3];
  /**< Vector from the origin of the estimation frame to the origin of the launch frame represented with respect to the estimation frame. (Units: m) */
  float yaw_el;
  /**< Yaw angle of the launch frame with respect to the estimation frame. (Units: rad) */
  uint8_t launch_tf_is_valid;
  /**< Indicates if the transformation between the estimation frame and the launch frame is valid or not. */
} PosVel;

/**
 * VIO position and velocity control data.
 */
typedef struct
{
  int64_t time;
  /**< Timestamp.  (Units: @us) */
  uint32_t cntr;
  /**< Counter that is incremented with each control loop. */
  float position_estimated[3];
  /**< Estimated XYZ position. (Units: m) */
  float velocity_estimated[3];
  /**< Estimated XYZ velocity. (Units: m/s) */
  float yaw_estimated;
  /**< Estimated yaw angle.  (Units: rad) */
  float position_desired[3];
  /**< Desired XYZ position. (Units: m) */
  float velocity_desired[3];
  /**< Desired XYZ velocity. (Units: m/s) */
  float yaw_desired;
  /**< Desired yaw angle.  (Units: rad) */
  uint8_t is_valid;
  /**< Is 1 if the VIO position and velocity data is valid. */
} VioPosVel;

/**
 * GPS position and velocity control data.
 */
typedef struct
{
  int64_t time;
  /**< Timestamp.  (Units: @us) */
  uint32_t cntr;
  /**< Counter that is incremented with each control loop. */
  int32_t identifier;
  /**< Cast to enum: #SnGnssReceiverType. Type of GNSS receiver. */
  float position_estimated[3];
  /**< Estimated XYZ position. +X is east, +Y is north, +Z is vertically up  (Units: m) */
  float velocity_estimated[3];
  /**< Estimated XYZ velocity.  (Units: m/s) */
  float yaw_estimated;
  /**< Estimated yaw angle of the vehicle’s body-fixed frame with respect to the East North Up (ENU) frame.  (Units: rad) */
  float position_desired[3];
  /**< Desired XYZ position.  (Units: m) */
  float velocity_desired[3];
  /**< Desired XYZ velocity.  (Units: m/s) */
  float yaw_desired;
  /**< Desired yaw angle of the vehicle’s body-fixed frame with respect to the ENU frame.  (Units: rad) */
  uint8_t is_enabled;
  /**< If enabled (set to 1), this data is populated when the vehicle gets a GPS lock. */
} GpsPosVel;

/**
 * Optic flow position and velocity control data.
 */
typedef struct
{
  int64_t time;
  /**< Timestamp.  (Units: @us) */
  uint32_t cntr;
  /**< Number of times the data was read. */
  float position_estimated[3];
  /**< Estimated XYZ position.  (Units: m) */
  float velocity_estimated[3];
  /**< Estimated XYZ velocity.  (Units: m/s) */
  float yaw_estimated;
  /**< Estimated yaw angle.  (Units: rad) */
  float position_desired[3];
  /**< Desired XYZ position.  (Units: m) */
  float velocity_desired[3];
  /**< Desired XYZ velocity.  (Units: m/s) */
  float yaw_desired;
  /**< Desired yaw angle.  (Units: rad) */
  uint8_t is_valid;
  /**< 1 if the optic flow position and velocity data are valid, otherwise 0.  */
} OpticFlowPosVel;

/**
 * Raw ESC data.
 */
typedef struct
{
  uint32_t iter;
  /**< Loop iteration in which data was logged. */
  int64_t time;
  /**< Time at which this data was published. (Units: @us) */
  uint32_t bytes_tx;
  /**< Number of bytes received from ESCs. (Units: bytes) */
  uint32_t bytes_rx;
  /**< Number of bytes received from all ESCs. (Units: bytes) */
  uint32_t errors_rx;
  /**< Number of feedback read errors. */
  uint32_t packet_number;
  /**< Packet number. */
  uint8_t packet_cntr[8];
  /**< Number of control packets received by each ESC. */
  uint32_t packets_rx[8];
  /**< Number of packets received from each ESC. (Units: packets) */
  int16_t rpm[8];
  /**< Motor RPM. */
  int8_t power[8];
  /**< Power applied by the ESC.  (Units: %) */
  float voltage[8];
  /**< Voltage measured by each ESC.  (Units: V) */
  uint8_t states[8];
  /**< ESC state.  */
} EscRaw;

/**
 * VOA data.
 */
typedef struct
{
  int64_t time;
  /**< Time at which this data was received. (Units: @us) */
  uint32_t cntr;
  /**< DSP counter of data received. */
  uint64_t apps_proc_time;
  /**< Timestamp from the applications processor. (Units: @us) */
  uint32_t apps_proc_cntr;
  /**< Counter of data sent from the applications processor. */
  int num_points;
  /**< Number of points used for VOA control. */
} VoaData;

/**
 * State of VOA processing.
 */
typedef struct
{
  int64_t time;
  /**< Timestamp of information (Units: @us) */
  uint32_t cntr;
  /**< Monotonically increasing counter */
  uint8_t voa_enabled;
  /**< 1 if VOA is enabled, 0 otherwise. */
  uint8_t voa_running;
  /**< 1 if VOA is running and has the ability to modify control output, 0 otherwise. */
  uint8_t voa_active;
  /**< 1 if VOA is currently modifying control output, 0 otherwise. */
} VoaStatus;

/**
 * Raw obstacle distance information (if available).
 */
typedef struct
{
  int64_t time;
  /**< Timestamp of data. (Units: @us) */
  uint32_t cntr;
  /**< Data log counter. */
  float relative_distances[4];
  /**< Array of distances sensed in the body coordinate frame and centered around the forward +X direction. Each number corresponds to an angle specified in the parameter file. By default, each number corresponds to 0.3 radians. (Units: m) */
} RelativeObstacleDistances;

/**
 * GPS latitude and longitude at the time of initial GPS lock.
 */
typedef struct
{
  int64_t time;
  /**< Timestamp of initial GPS lock. (Units: @us) */
  int32_t origin_latitude;
  /**< Latitude at GPS lock. */
  int32_t origin_longitude;
  /**< Longitude at GPS lock. */
} GpsOrigin;

/**
 * Output from the mvVISLAM API transformed into the Snapdragon Navigator VIO frame, which is defined by the vehicle frame at VIO initialization. Field names are based on fields in the mvVISLAM API. Refer to the <a href="https://developer.qualcomm.com/software/machine-vision-sdk/tools\>Machine Vision SDK"</a> for more information about these fields.
 */
typedef struct
{
  uint32_t iter;
  /**< Loop iteration in which VIO data was logged. */
  int64_t time;
  /**< Time at which the DSP received VIO data (according to the DSP clock). (Units: @us) */
  int64_t apps_time;
  /**< Time at which the Apps Processor retrieved VIO data (according to the Apps Processor clock). (Units: @us) */
  int64_t pose_time;
  /**< Timestamp of the pose given by the mvVISLAM API. (Units: @us) */
  uint32_t cntr;
  /**< Update counter for this data structure. */
  int pose_quality;
  /**< Quality of the pose estimate. */
  float body_pose_position[3];
  /**< Vehicle position with respect to the Snapdragon Navigator VIO frame. (Units: m) */
  float body_pose_rotation[9];
  /**< Orientation of vehicle frame with respect to the Snapdragon Navigator VIO frame. */
  float err_cov_pose_position[9];
  /**< Error covariance matrix for vehicle position estimate. (Units: m^2) */
  float time_alignment;
  /**< Time misalignment between camera and IMU data. (Units: s) */
  float velocity[3];
  /**< Vehicle velocity with respect to the Snapdragon Navigator VIO frame. (Units: m/s) */
  float err_cov_velocity[9];
  /**< Error covariance matrix for the vehicle velocity estimate. (Units: (m/s)^2) */
  float angular_velocity[3];
  /**< Vehicle angular velocity with respect to the Snapdragon Navigator VIO frame. (Units: rad/s) */
  float gravity[3];
  /**< Gravity vector with respect to the Snapdragon Navigator VIO frame. (Units: m/s/s) */
  float err_cov_gravity[9];
  /**< Error covariance matrix for the gravity vector estimate. */
  float w_bias[3];
  /**< Gyro bias with respect to the Snapdragon Navigator VIO frame. (Units: rad) */
  float a_bias[3];
  /**< Accelerometer bias with respect to the Snapdragon Navigator VIO frame. (Units: m/s/s) */
  float tbc[3];
  /**< Translation from the camera to the IMU with respect to the Snapdragon Navigator VIO frame. (Units: m) */
  float Rbc[9];
  /**< Orientation of camera frame with respect to the IMU frame. */
  uint32_t error_code;
  /**< Error code of the mvVISLAM API. For information, refer to the Machine Vision SDK. */
  int32_t num_tracked_pts;
  /**< Number of map points being observed and estimated. */
} Vio0Raw;


typedef struct
{
  int64_t time;
  /**< Time at which the struct was logged. (Units: @us) */
  uint32_t loop_cntr;
  /**< Number of times the control loop has run */
  int16_t des_rpm[8];
  /**< Desired Motor RPMs when ESC is in RPM control mode (Units: rpm) */
  int16_t des_pwm[8];
  /**< Desired Motor PWMs when ESC is in PWM control mode (Units: pwm) */
} DesiredEscState;

/**
 * Raw world offset computation from fiducial markers.
 */
typedef struct
{
  uint64_t time;
  /**< Time at which the struct was logged.  (Units: @us) */
  uint32_t frame_id;
  /**< Camera frame ID used for marker detection. */
  int64_t frame_timestamp_ns;
  /**< Timestamp of the camera frame used for marker detection. (Units: ns) */
  uint32_t num_markers_used;
  /**< Number of marker detections in the frame used in offset computation. */
  float yaw_offset_raw;
  /**< Raw Yaw offset. (Units: rad) */
  float pos_offset_raw[3];
  /**< Raw XYZ offset (Units: m) */
} FiducialMarkerWorldOffsetRaw;

/**
 * Filtered world offset computation from fiducial markers.
 */
typedef struct
{
  uint64_t time;
  /**< Time at which the struct was logged.  (Units: @us) */
  uint32_t num_raw_offset_updates;
  /**< Number of times that the raw offsets were computed. */
  float pos_offset[3];
  /**< Vector from origin of marker world frame to origin of Snapdragon Navigator world frame represented in the Snapdragon Navigator world frame. (Units: m) */
  float yaw_offset;
  /**< Yaw component of the rotation from the Snapdragon Navigator world frame to the marker world frame. (Units: rad) */
  float pos_world[3];
  /**< Estimated position represented in the marker world frame. (Units: m) */
  float yaw_world;
  /**< Estimated yaw angle represented in the marker world frame. (Units: rad) */
} FiducialMarkerWorldOffsetData;

/**
 * Simulation ground truth.
 */
typedef struct
{
  int64_t time;
  /**< Sim current time. (Units: @us) */
  float position[3];
  /**< Ground truth position of the vehicle with respect to the simulation frame. (Units: m) */
  float velocity[3];
  /**< Ground truth velocity of the vehicle with respect to the simulation frame. (Units: m/s) */
  float R[9];
  /**< Ground truth orientation of the vehicle-fixed frame with respect to the simulation frame. */
} SimGroundTruth;

/**
 * Mag calibration data
 */
typedef struct
{
  int64_t time;
  /**< Timestamp.  (Units: @us) */
  uint32_t cntr;
  /**< Counter incremented upon data logged */
  float mag_data_raw[3];
  /**< x, y, z components of raw mag */
  float mag_att_est_R[9];
  /**< Att Est Rotation matrix */
} MagDataForCalibration;

/**
 * Mag matrix calibration data
 */
typedef struct
{
  int64_t time;
  /**< Timestamp.  (Units: @us) */
  uint32_t cntr;
  /**< Counter incremented upon data logged */
  float Winv_matrix[9];
  /**< Winv for mag cal */
  float offset_vector[3];
  /**< offset vector for mag cal */
  float Rotation_matrix[9];
  /**< Rot matrix for mag cal */
} MagCalibrationResults;

/**
 * Snapdragon Navigator cached data for the sn_get_flight_data_ptr() function.
 */
typedef struct
{
  VersionInfo version_info;
  /**< Version information required to uniquely identify the software and device. */
  MvSdkVersionInfo mv_sdk_version_info;
  /**< Machine vision (MV) version information. */
  SensorImuApiVersionInfo sensor_imu_api_version_info;
  /**< Version of the sensor_imu API. */
  GeneralStatus general_status;
  /**< General system state information for debugging system issues. */
  DataStatus data_status;
  /**< Status of various sensors and estimators. */
  UpdateRates update_rates;
  /**< System and sensor update rates. */
  AttitudeEstimate attitude_estimate;
  /**< Estimate of the vehicle orientation. */
  AttitudeEstimate1 attitude_estimate1;
  /**< Estimate of the vehicle orientation. */
  AttitudeEstimate2 attitude_estimate2;
  /**< Estimate of the vehicle orientation. */
  CpuStats cpu_stats;
  /**< Apps processor CPU status. */
  Imu0Raw imu_0_raw;
  /**< Inertial measurement unit 0 raw data. */
  Imu1Raw imu_1_raw;
  /**< Inertial measurement unit 1 raw data. */
  Imu2Raw imu_2_raw;
  /**< Inertial measurement unit 2 raw data. */
  Imu0Compensated imu_0_compensated;
  /**< Inertial measurement unit (IMU) data after compensation. */
  Imu1Compensated imu_1_compensated;
  /**< Inertial measurement unit (IMU) data after compensation. */
  Imu2Compensated imu_2_compensated;
  /**< Inertial measurement unit (IMU) data after compensation. */
  Imu0CalibrationThermal imu_0_calibration_thermal;
  /**< Data results from IMU temperature calibration. */
  Imu0CalibrationOffset imu_0_calibration_offset;
  /**< IMU sensor offset values. */
  Barometer0Raw barometer_0_raw;
  /**< Raw barometer data. */
  Sonar0Raw sonar_0_raw;
  /**< Raw sonar data. */
  Mag0Raw mag_0_raw;
  /**< Raw magnetometer data from the mag0 sensor. */
  Mag1Raw mag_1_raw;
  /**< Raw magnetometer data from the mag1 sensor. */
  Mag0Compensated mag_0_compensated;
  /**< Mag0 sensor data after compensation. */
  Mag1Compensated mag_1_compensated;
  /**< Mag1 sensor data after compensation. */
  Mag0Calibration3D mag_0_calibration_3d;
  /**< Data results from Mag0 3D calibration. */
  SpektrumRc0Raw spektrum_rc_0_raw;
  /**< Raw Spektrum RC data. */
  ApiRcRaw api_rc_raw;
  /**< RC commands sent through the API. */
  ApiThrustAttAngVel api_thrust_att_ang_vel;
  /**< Thrust, attitude, and angular velocity commands sent through the API. */
  ApiPropsCmd api_props_cmd;
  /**< Received API spin or stop propellers commands. */
  RcActive rc_active;
  /**< RC commands for control. */
  Camera0FrameInfo camera_0_frame_info;
  /**< Captured camera frame information from the downward camera. */
  OpticFlow0Raw optic_flow_0_raw;
  /**< Downward facing tracker (DFT) data. */
  OpticFlow0CalibrationTilt optic_flow_0_calibration_tilt;
  /**< Downfacing camera calibration for tilt angle (optic flow camera yaw calibration). */
  Gps0Raw gps_0_raw;
  /**< Raw GPS data. */
  Gps1Raw gps_1_raw;
  /**< Raw GPS data. */
  TrajectoryDataRaw trajectory_data_raw;
  /**< Raw API data for trajectory control input. */
  PosVel pos_vel;
  /**< Position and velocity control data. */
  VioPosVel vio_pos_vel;
  /**< VIO position and velocity control data. */
  GpsPosVel gps_pos_vel;
  /**< GPS position and velocity control data. */
  OpticFlowPosVel optic_flow_pos_vel;
  /**< Optic flow position and velocity control data. */
  EscRaw esc_raw;
  /**< Raw ESC data. */
  VoaData voa_data;
  /**< VOA data. */
  VoaStatus voa_status;
  /**< State of VOA processing. */
  RelativeObstacleDistances relative_obstacle_distances;
  /**< Raw obstacle distance information (if available). */
  GpsOrigin gps_origin;
  /**< GPS latitude and longitude at the time of initial GPS lock. */
  Vio0Raw vio_0_raw;
  /**< Output from the mvVISLAM API transformed into the Snapdragon Navigator VIO frame, which is defined by the vehicle frame at VIO initialization. Field names are based on fields in the mvVISLAM API. Refer to the <a href="https://developer.qualcomm.com/software/machine-vision-sdk/tools\>Machine Vision SDK"</a> for more information about these fields. */
  DesiredEscState desired_esc_state;
  FiducialMarkerWorldOffsetRaw fiducial_marker_world_offset_raw;
  /**< Raw world offset computation from fiducial markers. */
  FiducialMarkerWorldOffsetData fiducial_marker_world_offset_data;
  /**< Filtered world offset computation from fiducial markers. */
  SimGroundTruth sim_ground_truth;
  /**< Simulation ground truth. */
  MagDataForCalibration mag_data_for_calibration;
  /**< Mag calibration data */
  MagCalibrationResults mag_calibration_results;
  /**< Mag matrix calibration data */
} SnavCachedData;

/** @} */ /* end_addtogroup sn_datatypes */

#endif // SNAV_CACHED_DATA_H_
