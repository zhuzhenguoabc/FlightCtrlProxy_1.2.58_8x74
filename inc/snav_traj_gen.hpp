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

#include <vector>
#include <array>
#include "waypoint.hpp"

#ifndef _SNAV_TRAJ_GEN_HPP_
#define _SNAV_TRAJ_GEN_HPP_

namespace snav_traj_gen{
/** @ingroup traj_gen
 *
 * @brief SNAV Trajectories are polynomial-based, dynamically feasible trajectories
 * for SNAV vehicles. Trajectories are specified by series of waypoints.
 * These waypoints can have fixed or free positions, velocities, accelerations,
 * and jerk. libsnav_traj_gen will then optimize the trajectory to achieve minimum
 * snap, or another derivative specified.
 */
class SnavTrajectory{
public:
  /**
   * @brief Constructs an instance of SnavTrajectory
   */
  SnavTrajectory();

  /**
   * @brief Optimize trajectory with all options
   *
   * @param[in] input_wps Vector of waypoints specifying the trajectory
   * @param[in] deriv_num Derivative to minimize (4 = snap, 3 = jerk, 
   *                      2 = acceleration)
   * @param[in] loop      true if this trajectory should loop.  If
   *                      true, the last waypoint will be set equal
   *                      to the first, and it's "constrained" member
   *                      will be ignored
   * @return Cost of the trajectory (sum-squared deriv_num)
   *
   * @note optimize() must be called before calling sample(), 
   *       calculate_derivative_maximums(), or using the member
   *       coefficient arrays.  The populated coefficents are packed into
   *       the vectors SnavTrajectory::c_pos_x, SnavTrajectory::c_pos_y, 
   *       SnavTrajectory::c_vel_x, etc.  Each array of
   *       coeefficients represents the 7th order polynomial coefficients
   *       for one polynomial between two waypoints.  As such, there are 
   *       (num_waypoints-1) sets of coefficients.  The coefficients can be 
   *       evaluated using t_power, and use the following ordering
   *       c[0] + c[1]*t + c[2]*t^2 + c[3]*t^3 + c[4]*t^4 + 
   *       c[5]*t^5 + c[6]*t^6 + c[7]*t^7.  The function also populates
   *       SnavTrajectory::optimized_wps, filling any unconstrained states
   *       with the optimized values.
   */  
  float optimize(std::vector<Waypoint> input_wps, int deriv_num, bool loop);

  /**
   * @brief Overloaded optimize() function with less options
   *
   * @param[in] input_wps Vector of waypoints specifying the trajectory
   *
   * @return Cost of the trajectory (sum-squared deriv_num)
   */
  float optimize(std::vector<Waypoint> input_wps);
  
  /**
   * @brief Sample of the optimized trajectory to query state at given time
   *
   * @param[in]  time   Time in seconds to query state on trajectory
   * @param[out] output Full desired state from trajectory at given time
   */
  int sample(StateVector &output, float time);

  /**
   * @brief Helper function for manual calculation of state from coefficient
   *        vectors.  Constructs an array of: 
   *        [1 t t^2 t^3 t^4 t^5 t^5 t^6 t^7]
   *        The dot product of this array and the polynomials coefficients
   *        evaluate to the state for that polynomial.
   *        For example:
   *        std::array<float,8> tpow = t_power(poly_t);
   *        float x_position = std::inner_product(tpow.begin(),tpow.end(),
   *                                              c_pos_x[i].begin(),0.0);
   *
   * @param[in]  time   Time in seconds
   */  
  std::array<float,8> t_power(float time);

  /**
   * @brief Samples the trajectory at granularity: dt. This populates the
   *        member variables max_*
   *        These include maximum velocities, accelerations, and jerk in
   *        x, y, z, the x-y plane norm, and the x-y-z norm. 
   *
   * @param[in]  dt Sampling timestep in seconds for calculating maximums
   */    
  void calculate_derivative_maximums(float dt);
  float max_velocity_x; /**< Maximum (absolute value) X Velocity [m/s]**/
  float max_velocity_y; /**< Maximum (absolute value) Y Velocity [m/s]**/
  float max_velocity_z; /**< Maximum (absolute value) Z Velocity [m/s]**/
  float max_velocity_xy; /**< Maximum norm X-Y Velocity [m/s]**/
  float max_velocity_xyz; /**< Maximum norm X-Y-Z Velocity [m/s]**/
  float max_acceleration_x; /**< Maximum (absolute value) X Acceleration [m/s^2]**/
  float max_acceleration_y; /**< Maximum (absolute value) Y Acceleration [m/s^2]**/
  float max_acceleration_z; /**< Maximum (absolute value) Z Acceleration [m/s^2]**/
  float max_acceleration_xy; /**< Maximum norm X-Y Acceleration [m/s^2]**/
  float max_acceleration_xyz; /**< Maximum norm X-Y-Z Acceleration [m/s^2]**/
  float max_jerk_x; /**< Maximum (absolute value) X Jerk [m/s^3]**/
  float max_jerk_y; /**< Maximum (absolute value) Y Jerk [m/s^3]**/
  float max_jerk_z; /**< Maximum (absolute value) Z Jerk [m/s^3]**/
  float max_jerk_xy; /**< Maximum norm X-Y Jerk [m/s^3]**/
  float max_jerk_xyz; /**< Maximum norm X-Y-Z Jerk [m/s^3]**/
  
  std::vector<Waypoint> optimized_wps; /**< Updated waypoints after optimization.
                                            Any unconstrained states are populated
                                            with the optimized values **/
  
  std::vector<std::array<float,8>> c_pos_x; /**< X Position Coefficients:
                                               p_x(t)=dot(t_power(),c_pos_x) [m] **/
  std::vector<std::array<float,8>> c_pos_y; /**< Y Position Coefficients:
                                               p_y(t)=dot(t_power(),c_pos_y) [m] **/
  std::vector<std::array<float,8>> c_pos_z; /**< Z Position Coefficients:
                                               p_z(t)=dot(t_power(),c_pos_z) [m] **/
  std::vector<std::array<float,8>> c_vel_x; /**< X Velocity Coefficients:
                                               v_x(t)=dot(t_power(),c_vel_x) [m/s] **/
  std::vector<std::array<float,8>> c_vel_y; /**< Y Velocity Coefficients:
                                               v_y(t)=dot(t_power(),c_vel_y) [m/s] **/
  std::vector<std::array<float,8>> c_vel_z; /**< Z Velocity Coefficients:
                                               v_z(t)=dot(t_power(),c_vel_z) [m/s] **/
  std::vector<std::array<float,8>> c_acc_x; /**< X Acceleration Coefficients:
                                               a_x(t)=dot(t_power(),c_acc_x) [m/s^2] **/
  std::vector<std::array<float,8>> c_acc_y; /**< Y Acceleration Coefficients:
                                               a_y(t)=dot(t_power(),c_acc_y) [m/s^2] **/
  std::vector<std::array<float,8>> c_acc_z; /**< Z Acceleration Coefficients:
                                               a_z(t)=dot(t_power(),c_acc_z) [m/s^2] **/
  std::vector<std::array<float,8>> c_jrk_x; /**< X Jerk Coefficients:
                                               j_x(t)=dot(t_power(),c_jrk_x) [m/s^3] **/
  std::vector<std::array<float,8>> c_jrk_y; /**< Y Jerk Coefficients:
                                               j_y(t)=dot(t_power(),c_jrk_y) [m/s^3] **/
  std::vector<std::array<float,8>> c_jrk_z; /**< Z Jerk Coefficients:
                                               j_z(t)=dot(t_power(),c_jrk_z) [m/s^3]**/
private:
  bool loop;
};
}
#endif
