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

#ifndef WAYPOINT_HPP_
#define WAYPOINT_HPP_

#include <array>
#include "state_vector.hpp"

namespace snav_traj_gen
{

/** @ingroup traj_gen_datatypes
 * @brief This class represents a state with a timestamp and associated
 * metadata
 */
class Waypoint : public StateVector
{
public:

  /**
   * @brief Construct a waypoint with default state (set to zeros)
   */
  Waypoint()
  {
    constrained.fill(true);
    time = 0.0;
  }

  float time;                          /**< Timestamp [s] **/
  std::array<bool, 4> constrained; /**< Boolean vector specifying whether
                                         the derivative is fixed **/
};

} // namespace snav_traj_gen

#endif // WAYPOINT_HPP_

