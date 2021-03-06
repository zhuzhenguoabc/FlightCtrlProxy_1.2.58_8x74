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

#ifndef SNAV_TYPES_H_
#define SNAV_TYPES_H_

/** @addtogroup sn_datatypes
@{ */
/**
 * Mode ID codes returned by querying the flight system mode.
 */
typedef enum
{
  SN_SENSOR_ERROR_MODE = -2,
  /**< Error -- flight is not possible in current state. */
  SN_UNDEFINED_MODE = -1,
  /**< Mode is not defined in this list. */
  SN_WAITING_FOR_DEVICE_TO_CONNECT,
  /**< Waiting for an RC or DroneController to connect. */
  SN_EMERGENCY_KILL_MODE,
  /**< Propellers were stopped -- Most likely due to a crash. */
  SN_EMERGENCY_LANDING_MODE,
  /**< Low fixed-thrust emergency descent. */
  SN_THERMAL_IMU_CALIBRATION_MODE,
  /**< Thermal accel/gyro calibration. */
  SN_STATIC_ACCEL_CALIBRATION_MODE,
  /**< Static accel offset calibration. */
  SN_OPTIC_FLOW_CAM_YAW_CALIBRATION_MODE,
  /**< Optic flow camera yaw calibration. */
  SN_MAGNETOMETER_CALIBRATION_MODE,
  /**< Compass (magnetometer) calibration. */
  SN_CALIBRATION_SUCCESS,
  /**< Last active calibration was successful. */
  SN_CALIBRATION_FAILURE,
  /**< Last active calibration was not successful. */
  SN_ESC_RPM_MODE,
  /**< API controls the ESC RPMs. */
  SN_ESC_PWM_MODE,
  /**< API controls the ESC PWMs. */
  SN_RATE_MODE,
  /**< Thrust, roll rate, pitch rate, yaw rate; does not auto-stabilize. */
  SN_THRUST_ANGLE_MODE,
  /**< Thrust, roll angle, pitch angle, yaw rate. */
  SN_ALT_HOLD_MODE,
  /**< Vertical velocity, roll angle, pitch angle, yaw rate. */
  SN_THRUST_GPS_HOVER_MODE,
  /**< Thrust control with lateral position hold using GPS.  */
  SN_GPS_POS_HOLD_MODE,
  /**< Body-relative 3D velocity and yaw rate using GPS. */
  SN_OPTIC_FLOW_POS_HOLD_MODE,
  /**< Body-relative 3D velocity and yaw rate using optic flow. */
  SN_VIO_POS_HOLD_MODE,
  /**< Body-relative 3D velocity and yaw rate using VIO. */
  SN_THRUST_ATT_ANG_VEL_MODE,
  /**< Thrust, attitude, and angular velocity. */
  SN_PRESSURE_LANDING_MODE,
  /**< Vertical velocity-controlled descent with zero roll/pitch. */
  SN_PRESSURE_GPS_LANDING_MODE,
  /**< 3D velocity-controlled descent. */
  SN_GPS_GO_HOME_MODE,
  /**< 3D velocity-controlled return to home position. */
  SN_ALT_HOLD_LOW_ANGLE_MODE,
  /**< Vertical velocity, roll angle, pitch angle, and yaw rate with
   * limits on roll and pitch angles. */
  SN_POS_HOLD_MODE,
  /**< Body-relative 3D velocity and yaw rate. */
  SN_POS_LANDING_MODE,
  /**< 3D velocity-controlled descent using the best sensors available. */
} SnMode;

/**
 * Input command types.
 */
typedef enum
{
  SN_INPUT_CMD_TYPE_NONE = -1,
  /**< No input. */
  SN_INPUT_CMD_TYPE_RC,
  /**< RC-style input commands. */
  SN_INPUT_CMD_TYPE_API_THRUST_ATT_ANG_VEL,
  /**< Thrust attitude angular velocity input commands from the API. */
  SN_INPUT_CMD_TYPE_API_ESC,
  /**< ESC input commands from the API. */
  SN_INPUT_CMD_TYPE_API_TRAJECTORY_CONTROL,
  /**< Trajectory control commands from the API. */
} SnInputCommandType;

/**
 * RC command input source.
 */
typedef enum
{
  SN_RC_CMD_NO_INPUT = -1,
  /**< No input. */
  SN_RC_CMD_SPEKTRUM_INPUT,
  /**< Spektrum. */
  SN_RC_CMD_API_INPUT,
  /**< RC commands from API. */
} SnRcCommandSource;

/**
 * RC command. This enum specifies how the dimensionless commands sent by the
 * sn_send_rc_command() function are interpreted and indirectly selects
 * the desired operation mode. The actual mode can be verified with the
 * sn_get_mode() function.
 */
typedef enum
{
  SN_RC_RATES_CMD,
  /**< Command pitch rate, negative roll rate, thrust magnitude, and yaw rate. */
  SN_RC_THRUST_ANGLE_CMD,
  /**< Command pitch angle, negative roll angle, thrust magnitude, and yaw rate. */
  SN_RC_ALT_HOLD_CMD,
  /**< Command pitch angle, negative roll angle, Z speed, and yaw rate. */
  SN_RC_THRUST_ANGLE_GPS_HOVER_CMD,
  /**< Command pitch angle, negative roll angle, thrust magnitude, and yaw rate;
       holds lateral position using GPS when roll and pitch commands are zero. */
  SN_RC_GPS_POS_HOLD_CMD,
  /**< Command vehicle-relative X and Y speeds, Z speed, and yaw rate using GPS. */
  SN_RC_OPTIC_FLOW_POS_HOLD_CMD,
  /**< Command vehicle-relative X and Y speeds, Z speed, and yaw rate using optic
       flow. */
  SN_RC_VIO_POS_HOLD_CMD,
  /**< Command vehicle-relative X and Y speeds, Z speed, and yaw rate using
       visual inertial odometry (VIO). */
  SN_RC_ALT_HOLD_LOW_ANGLE_CMD,
  /**< Command pitch angle, negative roll angle, Z speed, and yaw rate with a
       maximum tilt angle limit. */
  SN_RC_POS_HOLD_CMD,
  /**< Command vehicle-relative X and Y speeds, Z speed, and yaw rate using
       any available sensors. */
  SN_RC_NUM_CMD_TYPES
  /**< Do not use -- Reserved to hold the number of RC command types. */
} SnRcCommandType;


/**
 * Position contoller. This enum contains supported position controllers to specify how to interpret position, angle, and their derivatives into the sn_send_trajectory_tracking_command() function.
 * Each position controller uses an appropriate estimate of position and programs that use these controllers need to ensure that the correct reference frame is used.
 */
typedef enum
{
  SN_POSITION_CONTROL_GPS = 0,
  /**< GPS-based position control. */
  SN_POSITION_CONTROL_VIO,
  /**< VIO-based position control. */
  SN_POSITION_CONTROL_OF,
  /**< Optic flow-based position control. */
  SN_POSITION_CONTROL_NUM_TYPES
  /**< Do not use -- Reserved for the number of position control types. @newpage */
} SnPositionController;


/**
 * Options for trajectory tracking to be used in the sn_send_trajectory_tracking_command() function.
 */
typedef enum
{
  SN_TRAJ_DEFAULT = 0,
  /**< Default options. */
  SN_TRAJECTORY_OPTIONS_NUM
  /**< Do not use -- Reserved for the number of trajectory options. */
} SnTrajectoryOptions;


/**
 * RC command options.
 * The options can be OR-ed to form hybrid options
 */
typedef enum
{
  RC_OPT_LINEAR_MAPPING          =0,
  /**< Linear control (default). */
  RC_OPT_ENABLE_DEADBAND         =1,
  /**< Enable deadband. */
  RC_OPT_COMPLIANT_TRACKING      =2,
  /**< Enables the flight controller to modify and smooth input commands for feasibility.
     Obstacle avoidance features require this bit to be set, but commands might not
     not be tracked precisely if this flag is set. Use this flag when stick
     inputs are used and disable it to track motion precisely. */
  RC_OPT_DEFAULT_RC              =3,
  /**< Default RC.  */
  RC_OPT_TRIGGER_LANDING         =8,
  /**< Trigger landing. The vehicle determines which landing mode is appropriate
       based on which sensors are available and what mode is active. */
} SnRcCommandOptions;

/**
 * Collective state of all of the propellers -- Identification code returned by
 * querying the flight system propeller state.
 */
typedef enum
{
  SN_PROPS_STATE_UNKNOWN = -1,   /**< State of propellers is unknown. */
  SN_PROPS_STATE_NOT_SPINNING,   /**< All propellers are not spinning. */
  SN_PROPS_STATE_STARTING,       /**< Propellers are starting to spin. */
  SN_PROPS_STATE_SPINNING        /**< All propellers are spinning. */
} SnPropsState;

/**
 * Identification code returned by querying the sensor data status.
 */
typedef enum
{
  SN_DATA_INVALID = -1,
  /**< Sensor data is invalid. */
  SN_DATA_VALID = 0,
  /**< Sensor data is valid. */
  SN_DATA_NOT_INITIALIZED,
  /**< Sensor data has not been initialized. */
  SN_DATA_STUCK,
  /**< Sensor data is unchanging. */
  SN_DATA_TIMEOUT,
  /**< Sensor data has not been updated past the data timeout threshold. */
  SN_DATA_UNCALIBRATED,
  /**< Sensor data has not been calibrated. */
  SN_DATA_OFFSET_UNCALIBRATED,
  /**< Sensor data is missing offset calibration. */
  SN_DATA_TEMP_UNCALIBRATED,
  /**< Sensor data is missing temperature calibration. */
  SN_DATA_STARTING,
  /**< Sensor is acquiring additional samples. */
  SN_DATA_STATUS_UNAVAILABLE,
  /**< Sensor data status unavailable. */
  SN_DATA_NOT_ORIENTED,
  /**< Sensor data missing the orientation parameter. */
  SN_DATA_NO_LOCK,
  /**< Sensor data unable to lock on. */
  SN_DATA_WARNING,
  /**< Sensor is in a warning state. */
  SN_DATA_TRANSITIONING,
  /**< Sensor data is transitioning. */
  SN_DATA_LOCK_CONVERGING
  /**< Sensor data pending lock. */
} SnDataStatus;

/**
 * Individual state of a motor -- Identification code returned by querying the
 * state feedback from ESCs.
 */
typedef enum
{
  SN_MOTOR_STATE_UNKNOWN = -1,      /**< State of motor is unknown. */
  SN_MOTOR_STATE_NOT_SPINNING,      /**< Motor is not spinning. */
  SN_MOTOR_STATE_STARTING,          /**< Motor is starting to spin. */
  SN_MOTOR_STATE_SPINNING_FORWARD,
  /**< Motor is spinning in the forward direction. */
  SN_MOTOR_STATE_SPINNING_BACKWARD
  /**< Motor is spinning in the backward direction. */
} SnMotorState;

/**
 * Identification code returned by querying the sensor calibration status.
 */
typedef enum
{
  SN_CALIB_STATUS_NOT_CALIBRATED,
  /**< Calibration data does not exist. */
  SN_CALIB_STATUS_CALIBRATION_IN_PROGRESS,
  /**< Calibration procedure is in progress. */
  SN_CALIB_STATUS_CALIBRATED
  /**< Calibration data exists. */
} SnCalibStatus;

/**
 * Spektrum data transmission mode. This value is used to request binding and display the current mode.
 */
typedef enum
{
  SN_RC_RECEIVER_MODE_UNKNOWN,
  /**< Unknown DSM mode. */
  SN_SPEKTRUM_MODE_DSM2_22,
  /**< DSM2 22 ms (6-channel maximum, every 22 ms). */
  SN_SPEKTRUM_MODE_DSM2_11,
  /**< DSM2 11 ms (9-channel maximum, complete packet every 22 ms).  */
  SN_SPEKTRUM_MODE_DSMX_22,
  /**< DSMX 22 ms (6-channel maximum, every 22 ms). */
  SN_SPEKTRUM_MODE_DSMX_11
  /**< DSMX 11 ms (9-channel maximum, complete packet every 22 ms). @newpage */
} SnRcReceiverMode;


/**
 * Supported GNSS receiver types.
 */
typedef enum
{
  SN_GNSS_RECEIVER_TYPE_UNKNOWN = 0,
  /**< Unknown receiver. */
  SN_GNSS_RECEIVER_TYPE_CSR_SSV,
  /**< CSR receiver. */
  SN_GNSS_RECEIVER_TYPE_QC_WGR,
  /**< Qualcomm WGR receiver. */
  SN_GNSS_RECEIVER_TYPE_UBLOX,
  /**< U-blox receiver.  */
} SnGnssReceiverType;

/**
 * Position estimate type.
 *
 * Multiple sensors can be used to estimate the position. This enum specifies
 * the dominant source of the estimate used to determine the expected performance
 * level.
 */
typedef enum
{
  SN_POS_EST_TYPE_NONE = -1,
  /**< No position estimate is available. */
  SN_POS_EST_TYPE_GPS,
  /**< GPS is the dominant source of position estimate. */
  SN_POS_EST_TYPE_VIO,
  /**< VIO is the dominant source of position estimate. */
  SN_POS_EST_TYPE_DFT,
  /**< Downward facing tracker (DFT) is the dominant source of position estimate. */
} SnPosEstType;

/**
 * Parameter set and query return codes
 *
 */
typedef enum
{
  SN_PARAM_FAILURE = -1,
  /**< Parameter get or set failed. Ensure SNAV is running. */
  SN_PARAM_SUCCESS = 0,
  /**< Parameter get or set operation successful. */
  SN_PARAM_PARENT_NOT_FOUND,
  /**< Parent does not exist. */
  SN_PARAM_PARAM_NOT_FOUND,
  /**< Parameter does not exist. */
  SN_PARAM_LOCKED,
  /**< Parameter cannot be changed dynamically. */
} SnParamRet;

/** @} */ /* end_addtogroup sn_datatypes */


#endif //SNAV_TYPES_H_
