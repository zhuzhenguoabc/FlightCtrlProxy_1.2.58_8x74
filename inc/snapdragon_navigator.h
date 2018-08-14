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

#ifndef SN_INTERFACE_H_
#define SN_INTERFACE_H_

#include "snav_cached_data.h"
#include "snav_types.h"


#ifndef __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE
#define __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE
#endif // __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE

#define VERSION "1.2.58"

#define DEPRECATED __attribute__((deprecated))

/** @file */

#ifdef __cplusplus
extern "C"{
#endif


/** @ingroup sn_interface
 * Updates the internal cache of flight control data.
 *
 * @detdesc
 * This function caches the current state of all other components that can be
 * queried. This function must be called once per control loop before querying
 * the flight software information. This function also handles initialization
 * of API assets and must be called at least once before calling any other
 * API function.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * None.
 * @newpage
 */
int sn_update_data() __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;

/** @addtogroup sn_interface
@{ */
/**
 * Non-blocking attempt to spin propellers.
 *
 * @detdesc
 * This function does not guarantee that propellers start spinning.
 * Instead, safety checks are performed and then propellers started if deemed
 * safe.
 * @par
 * This function introduces a time delay before the propellers spin.
 * @par
 * @note1hang For this function to have effect, the following conditions must
 *            be met:
 *            - Propellers must not be spinning -- Verify using the
 *              sn_get_props_state() function.
 *            - Vehicle must be in a flight mode, that is, a heartbeat
 *              must already be established by calling sn_send_rc_command() or
 *              a similar function.
 * @par
 * Check SnPropsState using the sn_get_props_state() function to verify that the
 * command executed.
 *
 * @return
 * - 0 if attempt was received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called at least once prior to calling
 * this function.
 */
int sn_spin_props() __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;

/**
 * Non-blocking attempt to stop propellers.
 *
 * @detdesc
 * This function does not guarantee that propellers stop spinning. Safety checks
 * are performed internally and propellers stop if deemed safe.
 * @par
 * @note1hang For this function to have effect, the following conditions must
 *            be met:
 *            - Propellers must be spinning or starting -- Verify using the
 *              sn_get_props_state() function.
 *            - Vehicle must be in a flight mode, that is, a heartbeat
 *              must already be established by calling sn_send_rc_command() or
 *              a similar function.
 * @par
 * Check SnPropsState using the sn_get_props_state() function to verify that the
 * command executed.
 *
 * @return
 * - 0 if attempt was received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called at least once prior to calling
 * this function.
 */
int sn_stop_props() __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;

/**
 * Non-blocking attempt to start static accelerometer calibration.
 *
 * @detdesc
 * Calibration only starts if it is deemed safe and appropriate for vehicle
 * to do so. Refer to <em>Qualcomm Snapdragon Navigator User Guide</em>
 * (80-P4698-18) for instructions.
 * @par
 * During this calibration, ensure vehicle is completely stationary on a level
 * surface. Use the information in the #GeneralStatus struct to determine whether
 * the calibration succeeds or fails.
 *
 * @par
 * @note1hang Vehicle must be rebooted after calibration to enable flight.
 *
 * @return
 * - 0 if attempt was received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called at least once prior to calling
 * this function.
 */
int sn_start_static_accel_calibration() __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;

/**
 * Gets the static accelerometer calibration status from the internal cache
 * of flight control data. This function can be used to determine if calibration
 * data exists or if the calibration procedure is in progress.
 *
 * @datatypes
 * #SnCalibStatus
 *
 * @param[out] status
 * Pointer to value to be set to the status of the static accelerometer calibration.
 *
 * @return
 * - 0 if attempt was received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called at least once prior to calling
 * this function.
 */
int sn_get_static_accel_calibration_status(SnCalibStatus *status) __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;

/**
 * Non-blocking attempt to start dynamic accelerometer calibration.
 *
 * @detdesc
 * Calibration only starts if it is deemed safe and appropriate for vehicle
 * to do so.
 * @par
 * Refer to <em>Qualcomm Snapdragon Navigator User Guide</em> (80-P4698-18) for
 * instructions.
 * @par
 * Use the information in the #GeneralStatus struct to determine whether
 * the calibration succeeds or fails.
 *
 * @return
 * - 0 if attempt was received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called at least once prior to calling
 * this function.
 */
int sn_start_dynamic_accel_calibration() __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;

/**
 * Gets the status of dynamic accelerometer calibration from the internal cache
 * of flight control data. This function can be used to determine if calibration
 * data exists or if the calibration procedure is in progress.
 *
 * @datatypes
 * #SnCalibStatus
 *
 * @param[out] status Pointer to the value to be set to the status of the dynamic
 *                    accelerometer calibration.
 *
 * @return
 * - 0 if attempt was received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called at least once prior to calling
 * this function.
 */
int sn_get_dynamic_accel_calibration_status(SnCalibStatus *status) __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;

/**
 * Non-blocking attempt to start thermal IMU calibration.
 *
 * @detdesc
 * Calibration only starts if it is deemed safe and appropriate for vehicle
 * to do so. Refer to <em>Qualcomm Snapdragon Navigator User Guide</em>
 * (80-P4698-18) for instructions.
 * @par
 * During this calibration, ensure vehicle is completely stationary on a level
 * surface.
 * @par
 * Increase the vehicle temperature during this test to ensure that a large
 * temperature range observed.
 * @par
 * Use the information in the #GeneralStatus struct to determine whether
 * the calibration succeeds or fails.
 * @par
 * @note1hang The vehicle must be rebooted after calibration to enable flight.
 * @par
 * @note1hang A static calibration is required immediately after thermal
 *            calibration.
 *
 * @return
 * - 0 if attempt was received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called at least once prior to calling
 * this function.
 */
int sn_start_imu_thermal_calibration() __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;

/**
 * Gets the status of thermal IMU calibration from the internal cache of flight
 * control data. This function can be used to determine if calibration data exists
 * or if the calibration procedure is in progress.
 *
 * @datatypes
 * #SnCalibStatus
 *
 * @param[out] status
 * Pointer to the value to be set to the
 * status of the IMU thermal calibration.
 *
 * @return
 * - 0 if attempt was received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called at least once prior to calling
 * this function.
 */
int sn_get_imu_thermal_calibration_status(SnCalibStatus *status) __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;

/**
 * Non-blocking attempt to start optic flow camera yaw calibration.
 *
 * @detdesc
 * Calibration only starts if it is deemed safe and appropriate for the
 * vehicle. Refer to <em>Qualcomm Snapdragon Navigator User Guide</em>
 * (80-P4698-18) for instructions on how to run this calibration.
 * @par
 * @note1hang
 * Vehicle must be rebooted after calibration to enable flight.
 *
 * @return
 * - 0 if attempt was received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called at least once prior to calling
 * this function.
 */
int sn_start_optic_flow_camera_yaw_calibration() __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;

/**
 * Gets the status of optic flow camera yaw calibration from the internal cache
 * of flight control data. This function can be used to determine if calibration
 * data exists or if the calibration procedure is in progress.
 *
 * @datatypes
 * #SnCalibStatus
 *
 * @param[out] status
 * Pointer to the value to be set to the status of the optic flow camera
 * yaw calibration.
 *
 * @return
 * - 0 if attempt was received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called at least once prior to calling
 * this function. @newpage
 */
int sn_get_optic_flow_camera_yaw_calibration_status(SnCalibStatus *status) __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;

/**
 * Non-blocking attempt to start magnetometer (compass) calibration.
 *
 * @detdesc
 * Calibration only starts if it is deemed safe and appropriate for the
 * vehicle. Refer to <em>Qualcomm Snapdragon Navigator User Guide</em>
 * (80-P4698-18) for instructions on how to run this calibration.
 * @par
 * @note1hang
 * The vehicle must be rebooted after calibration to enable flight.
 *
 * @return
 * - 0 if attempt was received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called at least once prior to calling
 * this function.
 */
int sn_start_magnetometer_calibration() __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;

/**
 * Gets the magnetometer calibration status from the internal cache of flight
 * control data. This function can be used to determine if calibration data exists
 * or if the calibration procedure is in progress.
 *
 * @datatypes
 * #SnCalibStatus
 *
 * @param[out] status
 * Pointer to the value to be set to the status of the magnetometer
 * calibration.
 *
 * @return
 * - 0 if attempt was received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called at least once prior to calling
 * this function. @newpage
 */
int sn_get_magnetometer_calibration_status(SnCalibStatus *status) __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;

/**
 * Sends RPM commands to the ESCs and requests feedback.
 *
 * @param[in] rpm_data Pointer to the array containing RPM data to be sent to ESCs.
 *                     RPMs are ordered in ascending order of ESC ID, e.g.,
 *                     [rpm_0, rpm_1, ..., rpm_n]
 * @param[in] size     Number of rpm_data array elements.
 * @param[in] fb_id    ID of the ESC from which feedback is desired.
 *                     If fb_id = -1, no feedback is requested.
 *
 * @detdesc
 * Sending this command does not guarantee that the ESCs spin the motors.
 * If the vehicle is not in flight, the flight controller forwards the
 * RPM commands to the ESCs. The ESC identified by fb_id requests feedback.
 * @par
 * @note1hang RPM commands must be sent at a rate between 100 Hz and 500 Hz.
 *
 * @return
 * - 0 if command received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called at least once prior to calling
 * this function. @newpage
 */
int sn_send_esc_rpm(int *rpm_data, unsigned int size, int fb_id) __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;

/**
 * Sends PWM commands to the ESCs and requests feedback.
 *
 * @param[in] pwm_data Pointer to int array containing PWM data in the range
 *                     [-800, 800] to be sent to ESCs. PWMs are ordered in
 *                     ascending order of ESC ID, e.g. [pwm_0, pwm_1, ..., pwm_n].
 * @param[in] size     Number of pwm_data array elements.
 * @param[in] fb_id    ID of the ESC from which feedback is desired.
 *                     If fb_id = -1, no feedback is requested.
 *
 * @detdesc
 * The pwm_data array contains ESC PWMs in the range of [-800, 800] in which
 * 800 corresponds to 100% duty cycle and negative implies reversed direction.
 * @par
 * Sending this command does not guarantee that the ESCs spin the motors.
 * If the vehicle is not in flight, the flight controller forwards the
 * PWM commands to the ESCs. The ESC identified by fb_id requests feedback.
 * @par
 * @note1hang PWM commands must be sent at a rate between 100 Hz and 500 Hz.
 *
 * @return
 * - 0 if command received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called at least once prior to calling
 * this function. @newpage
 */
int sn_send_esc_pwm(int *pwm_data, unsigned int size, int fb_id) __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;

/**
 * Sends an "RC-like" command to the flight controller.
 *
 * @datatypes
 * #SnRcCommandType \n
 * #SnRcCommandOptions
 *
 * @param[in] type    Specification of the desired input interpretation.
 * @param[in] options Options for interpreting the command.
 * @param[in] cmd0    Value in the range [-1.0, 1.0] specifying a
 *                    "forward/backward" type command in which "forward" is
 *                     positive.
 * @param[in] cmd1    Value in the range [-1.0, 1.0] specifying a "left/right"
 *                    type command in which "left" is positive.
 * @param[in] cmd2    Value in the range [-1.0, 1.0] specifying an "up/down"
 *                    type command in which "up" is positive.
 * @param[in] cmd3    Value in the range [-1.0, 1.0] specifying a "rotate"
 *                    type command in which rotating counter-clockwise is positive.
 *
 * @detdesc
 * This function sends four dimensionless control commands to the flight
 * controller and establishes a heartbeat for the API application.
 * The interpretation of the four commands depends on the mode, which
 * can be obtained from the current_mode field of the #GeneralStatus struct.
 * The desired meaning of the
 * four commands is specified with the type parameter. A heartbeat must be
 * established by calling this function before the flight controller allows
 * the API to take control, such as to start spinning the propellers.
 * @par
 * See Section @xref{sec:Understand_RCCI} for the meaning of the four
 * commands in different contexts.
 * @par
 * Suggested #SnRcCommandOptions option usage:
 * - RC_OPT_DEFAULT_RC for intuitive joystick control,
 *   including a small deadband to prevent drift and more intuitive mapping
 * - RC_OPT_LINEAR_MAPPING for absolute control of outputs -- Useful with the
 *   sn_apply_cmd_mapping() function
 * @par
 * @note1hang RC commands must be sent at a rate of at least 50 Hz.
 *
 * @return
 * - 0 if command received
 * - -1 for failure (flight software non-functional)
 * - -2 if any command is NaN
 *
 * @dependencies
 * The sn_update_data() function must be called at least once prior to calling
 * this function.
 */
int sn_send_rc_command(SnRcCommandType type, SnRcCommandOptions options,
    float cmd0, float cmd1, float cmd2, float cmd3) __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;

/**
 * Converts dimensioned commands into dimensionless commands.
 *
 * @datatypes
 * #SnRcCommandType \n
 * #SnRcCommandOptions
 *
 * @param[in] type    Command type remapped to match the type used subsequently in
 *                    the sn_send_rc_command() function.
 *
 * @param[in] options Options to apply during mapping. See sn_send_rc_command().
 * @param[in] input0  Dimensioned command to be mapped into cmd0.
 * @param[in] input1  Dimensioned command to be mapped into cmd1.
 * @param[in] input2  Dimensioned command to be mapped into cmd2.
 * @param[in] input3  Dimensioned command to be mapped into cmd3.
 * @param[out] cmd0   Mapped unitless command 0 to be sent with the
 *                    sn_send_rc_command() function.
 * @param[out] cmd1   Mapped unitless command 1 to be sent with the
 *                    sn_send_rc_command() function.
 * @param[out] cmd2   Mapped unitless command 2 to be sent with the
 *                    sn_send_rc_command() function.
 * @param[out] cmd3   Mapped unitless command 3 to be sent with the
 *                    sn_send_rc_command() function.
 *
 * @detdesc
 * This function remaps commands with real units into the appropriate
 * dimensionless commands to be sent with sn_send_rc_command() function. Mapping
 * is based on on the type and options parameters.
 * See Section @xref{sec:Understand_RCCI} for more information.
 *
 * @return
 * - 0 if command received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * None.
 *
 * @sa
 * sn_send_rc_command()
 */
int sn_apply_cmd_mapping(SnRcCommandType type, SnRcCommandOptions options,
    float input0, float input1, float input2, float input3,
    float *cmd0, float *cmd1, float *cmd2, float *cmd3) __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;



/**
 * Gets a human-readable string associated with a specific enum type and value.
 *
 * @param[in] type  String specifying the type of enum value provided. For example,
 *                  "SnMode", "SnMotorState", etc.
 * @param[in] value Value to be converted to a string (based on the type provided).
 *
 * @return
 * Pointer to the string corresponding to the provided enum.
 *
 * @dependencies
 * None.
 */
const char * sn_get_enum_string(const char* type, int value) __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;

/**
 * Gets the the name of an RC command from the type.
 *
 * @param[in] type RC command type of interest.
 *
 * @return
 * Pointer to the command name.
 *
 * @dependencies
 * None. @newpage
 */
const char * sn_get_cmd_name(SnRcCommandType type) __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;

/**
 * Gets the the dimensioned units of an RC command from the type and index.
 *
 * @datatypes
 * #SnRcCommandType
 *
 * @param[in] type  RC command type of interest.
 * @param[in] index Index in range [0, 3] corresponding to cmd0 through cmd3.
 *
 * @return
 * Pointer to the units of a particular command.
 *
 * @dependencies
 * None.
 */
const char * sn_get_dimensioned_units(SnRcCommandType type, int index) __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;

/**
 * Gets the the minimum value in real units for an RC command from the type and
 * index.
 *
 * @datatypes
 * #SnRcCommandType
 *
 * @param[in] type  RC command type of interest.
 * @param[in] index Index in range [0,3] corresponding to cmd0 through cmd3.
 *
 * @detdesc
 * This function returns the smallest possible command to be applied to the system. The
 * returned value maps to a dimensionless value of -1.0.
 * @par
 * Use sn_get_dimensioned_units() to get a string descripton of the units.
 *
 * @return
 * Minimum-allowed value in real units.
 *
 * @dependencies
 * None.
 */
float sn_get_min_value(SnRcCommandType type, int index) __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;

/**
 * Gets the the maximum value in real units for an RC command from the type and
 * index.
 *
 * @datatypes
 * #SnRcCommandType
 *
 * @param[in] type  RC command type of interest.
 * @param[in] index Index in the range [0,3] corresponding to cmd0 through cmd3.
 *
 * @detdesc
 * This function returns the largest possible command to be applied to the system. This
 * value maps to a dimensionless value of 1.0.
 * @par
 * Use the sn_get_dimensioned_units() function to get a string descripton of the
 * units.
 *
 * @return
 * Maximum-allowed value in real units.
 *
 * @dependencies
 * None. @newpage
 */
float sn_get_max_value(SnRcCommandType type, int index) __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;

/**
 * Sends thrust, attitude, and angular velocity.
 *
 * @param[in] thrust Commanded thrust in grams.
 * @param[in] qw     Scalar component of quaternion.
 * @param[in] qx     X component of vector part of the quaternion.
 * @param[in] qy     Y component of vector part of the quaternion.
 * @param[in] qz     Z component of vector part of the quaternion.
 * @param[in] wx     X component of angular velocity in rad/s.
 * @param[in] wy     Y component of angular velocity in rad/s.
 * @param[in] wz     Z component of angular velocity in rad/s.
 *
 * @detdesc
 * This function sends the desired thrust in grams, desired attitude represented
 * as a quaternion, and the desired angular velocity vector in rad/s to the flight
 * controller.
 * @par
 * The quaternion is in the following form:
 * @par
 * q = qw + qx*i + qy*j + qz*k
 * @par
 * @note1hang Be cautious -- This function is for advanced users.
 *
 * @return
 * - 0 if command received
 * - -1 for failure (flight software non-functional)
 * - -2 if any input arguments are NaN
 *
 * @dependencies
 * The sn_update_data() function must be called at least once prior to calling
 * this function. @newpage
 */
int sn_send_thrust_att_ang_vel_command(float thrust, float qw, float qx,
    float qy, float qz, float wx, float wy, float wz) __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;

/**
 * Sends position, angle, and derivatives for advanced trajectory tracking.
 *
 * @param[in] controller Desired position controller.
 * @param[in] options    Options for trajectory tracking.
 * @param[in] x          X component of the position vector in the specified frame.
 * @param[in] y          Y component of the position vector in the specified frame.
 * @param[in] z          Z component of the position vector in the specified frame.
 * @param[in] xd         X component of the velocity vector in the specified frame.
 * @param[in] yd         Y component of the velocity vector in the specified frame.
 * @param[in] zd         Z component of the velocity vector in the specified frame.
 * @param[in] xdd        X component of the acceleration vector in the specified frame.
 * @param[in] ydd        Y component of the acceleration vector in the specified frame.
 * @param[in] zdd        Z component of the acceleration vector in the specified frame.
 * @param[in] yaw        Gravity-aligned yaw angle relative to the specified frame.
 * @param[in] yaw_rate   Rate at which the gravity-aligned yaw angle changes.
 *
 * @detdesc
 * This function allows precise control of the desired trajectories to use
 * feed-forward acceleration angles.
 * @note1hang This function is intended for advanced users -- A trajectory
 * planner must be used to update these options at a minimum of 50 Hz.
 *
 * @return
 * - 0 if command received
 * - -1 for failure (flight software non-functional)
 * - -2 if any input arguments are NaN
 * - -3 if the specified control mode is unavailable
 *
 * @dependencies
 * None. @newpage
 */
int sn_send_trajectory_tracking_command(SnPositionController controller, SnTrajectoryOptions options,
    float x,   float y,   float z,
    float xd,  float yd,  float zd,
    float xdd, float ydd, float zdd,
    float yaw, float yaw_rate) __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;

/**
 * Sets the battery voltage.
 *
 * @param[in] voltage Battery voltage (V).
 *
 * @detdesc
 * This function overrides the internal battery voltage estimate. This function
 * must be called at a rate faster than 5 Hz for the value to be considered valid,
 * otherwise the flight controller defaults to the internal estimate of
 * battery voltage.
 *
 * @return
 * - 0 if command received
 * - -1 for failure (flight software non-functional)
 * - -2 if the voltage is less than or equal to zero, or is NaN
 *
 * @dependencies
 * The sn_update_data() function must be called at least once prior to calling
 * this function.
 */
int sn_set_battery_voltage(float voltage) __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;


/**
 * Gets the pointer to the Snapdragon Navigator cached data structure.
 *
 * @datatypes
 * #SnavCachedData
 *
 * @param[in] size_cached_struct       Structure size. Ensures that the header
 *                                     file stays in sync. This argument must be
 *                                     sizeof(SnavCachedData).
 * @param[out] snav_cached_data_struct Pointer to be filled with the cached data
 *                                     structure pointer values. This structure is
 *                                     updated with a call to sn_update_data().
 *
 * @return
 * - 0 if flight data pointer returns successfully
 * - -1 for failure to get the pointer to flight data
 *
 * @dependencies
 * None.
 */
int sn_get_flight_data_ptr(int size_cached_struct, SnavCachedData **snav_cached_data_struct) __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;


/**
 * Sets the LED colors, overriding the Snapdragon Navigator LED colors.
 *
 * @param[in] led_colors_input_array Array of RGB triplets. The range for each value is 0-255.
 * @param[in] led_colors_size        Size of the input color array -- Value must be greater than zero, less than 25, and a multiple of 3.
 * @param[in] led_colors_timeout_us  Timeout in microseconds for Snapdragon Navigator to take over LED control after the API color commands stop.
 *
 * @detdesc
 * This function overrides the internal output of the LED colors. Currently only a single RGB triplet is used (first three bytes).
 * The timeout variable specifies the time in microseconds when the LED output switches back to Snapdragon Navigator control after the API color commands stop updating.
 * Color values are interpreted as binary (0 = Off, otherwise = On).
 *
 * @return
 * - 0 command received
 * - -1 critical failure (flight software is most likely non-functional)
 * - -2 bad length of the color data array
 * - -3 negative value provided as timeout
 *
 * @dependencies
 * The sn_update_data() function must be called at least once prior to calling
 * this function. @newpage
 */
int sn_set_led_colors(const uint8_t * led_colors_input_array, int led_colors_size, int led_colors_timeout_us) __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;

/**
 * Gets the ESC state feedback data from from the internal cache of flight control
 * data.
 *
 * @datatypes
 * #SnMotorState
 *
 * @param[out] state_feedback Pointer to the array to be filled with states from
 *                            ESCs. The array is filled in ascending order of ESC
 *                            IDs, e.g. [state_0, state_1, ..., state_n]
 * @param[in] size            Number of state_feedback array elements.
 * @param[out] used           Pointer to the value to be set to the number of
 *                            elements used of the state_feedback array
 *
 * @detdesc
 * The given array must have a number of elements equal to the number of
 * ESCs connected to the flight controller.
 * @par
 * If the given array is too small to hold all of the feedback data, no data
 * copies into the array and the function returns an error code.
 * @par
 * @note1hang ESC feedback data is only updated if feedback is requested. See the
 * sn_send_esc_rpm() and sn_send_esc_pwm() functions.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 * - -2 if the size of the array is not big enough to hold all of the feedback data
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @sa
 * sn_send_esc_rpm() \n
 * sn_send_esc_pwm()
 * @newpage
 */
int sn_get_esc_state_feedback(SnMotorState *state_feedback, unsigned int size,
    unsigned int *used) __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;

/**
 * @deprecated
 * This function will be removed in a future release.
 *
 * Query-estimated accelerometer biases.
 *
 * @param ax_bias Reference to the float to be filled with the X
 *                accelerometer-estimated bias in G's.
 * @param ay_bias Reference to the float to be filled with the Y
 *                accelerometer-estimated bias in G's.
 * @param az_bias Reference to the float to be filled with the Z
 *                accelerometer-estimated bias in G's.
 *
 * @detdesc
 * Biases are represented with respect to the flight controller's body frame. \n
 * Accelerometer biases are defined as follows: \n
 * compensated linear acceleration = raw linear acceleration - biases
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software is most likely non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to update these values. @newpage
 */
/** @cond */ DEPRECATED /** @endcond */ int sn_get_est_accel_bias(float *ax_bias,float *ay_bias,float *az_bias) __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;

/**
 * @deprecated
 * This function will be removed in a future release.
 *
 * Query-estimated gyroscope biases.
 *
 * @param wx_bias Reference to the float to be filled with the X
 *                gyro-estimated bias.
 * @param wy_bias Reference to the float to be filled with the Y
 *                gyro-estimated bias.
 * @param wz_bias Reference to the float to be filled with the Z
 *                gyro-estimated bias.
 *
 * @return
 * - 0 for success
 * - -1 for failure (most likely indicates flight non-functional software)
 *
 * @dependencies
 * The sn_update_data() function must be called to update these values.
 */
/** @cond */ DEPRECATED /** @endcond */ int sn_get_est_gyro_bias(float *wx_bias,float *wy_bias,float *wz_bias) __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;

/** @ingroup sn_interface
 * @deprecated
 * This function will be removed in a future release.
 *
 * Detects whether GPS is enabled.
 *
 * @param[out] gps_enabled
 * Pointer to GPS enable status -- 1 if GPS is enabled; 0 otherwise.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data. @newpage
 */
/** @cond */ DEPRECATED /** @endcond */ int sn_is_gps_enabled(int * gps_enabled) __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;

/**
 * Queries the value of a specified Snapdragon Navigator parameter.
 *
 * @datatypes
 * #SnParamRet
 *
 * @param[in] parent_name Null-terminated character array with the name of the parent structure containing the intended parameter.
 * @param[in] parameter_name Null-terminated character array with the name of the intended parameter.
 * @param[out] value_buffer Character buffer to be filled with the null-terminated value for the intended parameter if the query is successful, empty string otherwise.
 * @param[in] buffer_len Size of the input buffer in bytes.
 *
 * @detdesc
 * This function calls into the DSP to find the current value of a specified parameter. Values are returned as strings.
 *
 * @return
 * SnParamRet describing the state of the query operation.
 * - Success &ndash; SN_PARAM_SUCCESS
 * - Failure &ndash; Error codes to determine the reason for failed calls
 *
 * @dependencies
 * Snapdragon Navigator must be running. @newpage
 */
  SnParamRet sn_query_parameter(const char * parent_name, const char * parameter_name,  char* value_buffer, int buffer_len) __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;


/**
 * Sets the value of a specified Snapdragon Navigator parameter.
 *
 * @datatypes
 * #SnParamRet
 *
 * @param[in] parent_name Null-terminated character array with the name of the parent structure that contains the intended parameter.
 * @param[in] parameter_name Null-terminated character array with the name of the intended parameter.
 * @param[in] value Null-terminated character array with a string representing the intended parameter value.
 *
 * @detdesc
 * This function calls into the DSP to set the value of the specified parameter. Because this call is asynchronous with the control loop, the parameter is not set immediately. The parameter change request is checked for validity and put into a queue to be applied at the start of the next control loop.
 *
 * @return
 * SnParamRet describing the state of the queue operation.
 * - Success &ndash; SN_PARAM_SUCCESS
 * - Failure &ndash; Error codes to determine the reason for failed calls
 *
 * @note1hang Successful calls do not mean the parameter has changed immediately. To ensure the parameter has been set successfully, it must be queried using the sn_query_parameter() function after at least one full control loop. If errors occur, see the debug prints in the Snapdragon Navigator log.
 *
 * @dependencies
 * Snapdragon Navigator must be running. @newpage
 */
  SnParamRet sn_queue_parameter_change(const char * parent_name, const char * parameter_name,  const char* value) __SNAV_EXTERNAL_SYMBOL_ATTRIBUTE;


/** @} */ /* end_addtogroup sn_interface */

#ifdef __cplusplus
}
#endif

#endif //SN_INTERFACE_H_
