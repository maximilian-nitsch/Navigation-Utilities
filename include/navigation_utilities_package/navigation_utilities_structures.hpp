/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#pragma once

namespace navigation_utilities {

struct Wgs84Parameters {
  /**
   * @brief Semi-major axis of the ellipsoid (m).
   */
  double a;

  /**
   * @brief Semi-minor axis of the ellipsoid (m).
   */
  double b;

  /**
   * @brief Flattening of the ellipsoid (-).
   */
  double f;

  /**
   * @brief First eccentricity of the ellipsoid (-).
   */
  double e;

  /**
   * @brief Meridian radius of curvature (m).
   */
  double R_M;

  /**
   * @brief Prime vertical radius of curvature (m).
   */
  double R_N;

  /**
   * @brief Mean radius of curvature (m).
   */
  double R_0;

  /**
   * @brief Earth angular velocity (rad/s).
   */
  double omega_ie;
};

}  // namespace navigation_utilities