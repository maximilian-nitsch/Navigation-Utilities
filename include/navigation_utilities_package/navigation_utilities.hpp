/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#pragma once

#include <stdexcept>  // For std::invalid_argument

#include <Eigen/Dense>

#include "navigation_utilities_structures.hpp"

namespace navigation_utilities {

/**
 * Get the WGS84 parameters for the given geodetic coordinates.
 *
 * @details This function implements the WGS84 ellipsoid model parameters and
 * returns them as a structure. Depending on the input coordinates, the function
 * will return different WGS84 parameters. The parameters are:
 * - a: Semi-major axis of the ellipsoid
 * - b: Semi-minor axis of the ellipsoid
 * - f: Flattening of the ellipsoid
 * - e: First eccentricity of the ellipsoid
 * - R_M: Meridian radius of curvature
 * - R_N: Prime vertical radius of curvature
 * - R_0: Mean radius of curvature
 *
 * @see J. Farrell, Aided Navigation: GPS with High Rate Sensors, in Electronic
 * Engineering, New York: McGraw-Hill, 2008, p. 29-32.
 *
 * @param[in] posGeodetic Geodetic ECEF coordinates (latitude, longitude,
 * height) in (rad, rad, m) or (deg, deg, m) if is_input_in_degrees is true
 * @param[in] is_input_in_degrees If true, the geodetic input coordinates are in
 * degrees (default: false)
 *
 * @return WGS84 ellipsoid parameters structure
 */
Wgs84Parameters getWGS84Parameters(Eigen::Vector3d& posGeodetic,
                                   const bool is_input_in_degrees = false);

/**
 * Convert geodetic ECEF coordinates to Cartesian ECEF coordinates.
 *
 * @details This function implements an algorithm to convert geodetic
 * coordinates (latitude, longitude, height) to Cartesian ECEF coordinates (X,
 * Y, Z). The algorithm uses the WGS84 ellipsoid model.
 *
 * @see J. Farrell, Aided Navigation: GPS with High Rate Sensors, in Electronic
 * Engineering, New York: McGraw-Hill, 2008, p. 33.
 *
 * @param[in] posGeodetic Geodetic ECEF coordinates (latitude, longitude,
 * height) in (rad, rad, m) or (deg, deg, m) if is_input_in_degrees is true
 * @param[in] is_input_in_degrees If true, the geodetic input coordinates are in
 * degrees (default: false)
 *
 * @return Cartesian ECEF coordinates (X, Y, Z) in (m, m, m)
 */
Eigen::Vector3d convertEcefGeodeticToCartesian(
    Eigen::Vector3d& posGeodetic, const bool is_input_in_degrees = false);

/**
 * Convert Cartesian ECEF coordinates to geodetic ECEF coordinates.
 *
 * @details This function implements an algorithm to convert Cartesian
 * coordinates (X, Y, Z) to geodetic ECEF coordinates (latitude, longitude,
 * height). The algorithm uses the WGS84 ellipsoid model. It should be noted
 * that latitude and height are calculated iteratively. The literature suggests
 * at least 25 iterations for centimeter accuracy.
 *
 * @see J. Farrell, Aided Navigation: GPS with High Rate Sensors, in Electronic
 * Engineering, New York: McGraw-Hill, 2008, p. 34.
 *
 * @param[in] posCartesian Cartesian ECEF coordinates (X, Y, Z) in (m, m, m)
 * @param[in] convert_output_to_degrees If true, the geodetic output coordinates
 * are converted to degrees (default: false)
 *
 * @return Geodetic ECEF coordinates (latitude, longitude, height) in (rad, rad,
 * m) or (deg, deg, m) if convert_output_to_degrees is true
 */
Eigen::Vector3d convertEcefCartesianToGeodetic(
    const Eigen::Vector3d& posCartesian,
    const bool convert_output_to_degrees = false);

/**
 * @brief Convert geodetic ECEF coordinates to local navigation frame
 * coordinates.
 *
 * @details This function converts geodetic ECEF coordinates (latitude,
 * longitude, height) to local navigation (n) frame coordinates (north, east,
 * down). The origin (reference point) of the local navigation (n) frame is
 * provided in geodetic ECEF coordinates (latitude, longitude, height).
 *
 * @see P. D. Groves, Principles of GNSS, Inertial, and Multi-sensor
 * Integrated Navigation Systems, 2013, p. 76.
 *
 * @param[in] posGeodetic Geodetic ECEF coordinates (latitude, longitude,
 * height) in (rad, rad, m) or (deg, deg, m) if is_input_in_degrees is true
 * @param[in] posGeodeticReference Geodetic ECEF coordinates of the origin
 * (reference point) of the local navigation (n) frame in (rad, rad, m)
 * @param[in] is_input_in_degrees If true, the geodetic input coordinates are in
 * degrees (default: false)
 *
 * @return Local navigation (n) frame coordinates (north, east, down) in (m, m,
 * m)
 */
Eigen::Vector3d convertGeodeticEcefToNed(
    Eigen::Vector3d& posGeodetic, Eigen::Vector3d& posGeodeticReference,
    const bool is_input_in_degrees = false);

/**
 * @brief Convert local navigation frame coordinates to geodetic ECEF
 * coordinates.
 *
 * @details This function converts local navigation (n) frame coordinates
 * (north, east, down) to geodetic ECEF coordinates (latitude, longitude,
 * height). The origin (reference point) of the local navigation (n) frame is
 * provided in geodetic ECEF coordinates.
 *
 * @see P. D. Groves, Principles of GNSS, Inertial, and Multi-sensor Integrated
 * Navigation Systems, 2013, p. 76.
 *
 * @param[in] posNed Local navigation (n) frame coordinates (north, east, down)
 * in (m, m, m)
 * @param[in] posGeodeticReference Geodetic ECEF coordinates of the origin
 * (reference point) of the local navigation (n) frame in (rad, rad, m) or (deg,
 * deg, m) if convert_output_to_degrees is true
 * @param[in] is_input_in_degrees If true, the geodetic input coordinates are in
 * degrees (default: false)
 * @param[in] convert_output_to_degrees If true, the geodetic output coordinates
 * are converted to degrees (default: false)
 *
 * @return Geodetic ECEF coordinates (latitude, longitude, height) in (rad, rad,
 * m) or (deg, deg, m) if convert_output_to_degrees is true
 */
Eigen::Vector3d convertNedToGeodeticEcef(
    const Eigen::Vector3d& posNed, Eigen::Vector3d& posGeodeticReference,
    const bool is_input_in_degrees = false,
    const bool convert_output_to_degrees = false);

/**
 * @brief Convert local navigation frame standard deviations to geodetic
 * ECEF standard deviations.
 *
 * @details This function converts local navigation (n) frame standard
 * deviations (σ_north, σ_east, σ_down) to geodetic ECEF standard deviations
 * (σ_latitude, σ_longitude, σ_height). The origin (reference point) of the
 * local navigation (n) frame is provided in geodetic ECEF coordinates
 * (latitude, longitude, height).
 *
 * @see P. D. Groves, Principles of GNSS, Inertial, and Multi-sensor Integrated
 * Navigation Systems, 2013, p. TODO: add page number.
 *
 * @param[in] posNedStdDev Local navigation (n) frame standard deviations
 * (σ_north, σ_east, σ_down) in (m, m, m)
 * @param[in] posGeodeticReference Geodetic ECEF coordinates of the origin
 * (reference point) of the local navigation (n) frame in (rad, rad, m) or (deg,
 * deg, m) if convert_output_to_degrees is true
 * @param[in] is_input_in_degrees If true, the geodetic input coordinates are in
 * degrees (default: false)
 * @param[in] convert_output_to_degrees If true, the geodetic output coordinates
 * are converted to degrees (default: false)
 *
 * @return Geodetic ECEF standard deviations (σ_latitude, σ_longitude, σ_height)
 * in (rad, rad, m) or (deg, deg, m) if convert_output_to_degrees is true
 */
Eigen::Vector3d convertNedToGeodeticEcefStdDev(
    const Eigen::Vector3d& posNedStdDev, Eigen::Vector3d& posGeodeticReference,
    const bool is_input_in_degrees = false,
    const bool convert_output_to_degrees = false);

/**
 * @brief Convert local navigation frame covariance matrix to geodetic ECEF
 * frame covariance matrix.
 *
 * @details This function converts a covariance matrix from local navigation (n)
 * frame to geodetic ECEF frame covariance matrix. The origin (reference point)
 * of the local navigation (n) frame is provided in geodetic ECEF.
 *
 * @see P. D. Groves, Principles of GNSS, Inertial, and Multi-sensor Integrated
 * Navigation Systems, 2013, p. TODO: add page number.
 *
 * @param[in] posNedCovariance Local navigation (n) frame covariance matrix in
 * (m²) on the diagonals
 * @param[in] posGeodeticReference Geodetic ECEF coordinates of the origin
 * (reference point) of the local navigation (n) frame in (rad, rad, m) or (deg,
 * deg, m) if convert_output_to_degrees is true
 * @param[in] is_input_in_degrees If true, the geodetic input coordinates are in
 * degrees (default: false)
 *
 * @return Geodetic ECEF covariance matrix in (rad², rad², m²) on the diagonals
 */
Eigen::Matrix3d convertNedToGeodeticEcefCovariance(
    const Eigen::Matrix3d& posNedCovariance,
    Eigen::Vector3d& posGeodeticReference,
    const bool is_input_in_degrees = false);

/**
 * @brief Convert geodetic ECEF frame covariance matrix to local navigation
 * frame covariance matrix.
 *
 * @details This function converts a covariance matrix from geodetic ECEF
 * frame to local navigation (n) frame covariance matrix. The origin (reference
 * point) of the local navigation (n) frame is provided in geodetic ECEF.
 *
 * @see P. D. Groves, Principles of GNSS, Inertial, and Multi-sensor Integrated
 * Navigation Systems, 2013, p. TODO: add page number.
 *
 * @param[in] posGeodeticCovariance Geodetic ECEF frame covariance matrix in
 * (rad², rad², m²) on the diagonals
 * @param[in] posGeodeticReference Geodetic ECEF coordinates of the origin
 * (reference point) of the local navigation (n) frame in (rad, rad, m) or (deg,
 * deg, m) if convert_output_to_degrees is true
 * @param[in] is_input_in_degrees If true, the geodetic input coordinates are in
 * degrees (default: false)
 *
 * @return Local navigation frame covariance matrix in (m²) on the diagonals
 */
Eigen::Matrix3d convertGeodeticEcefToNedCovariance(
    const Eigen::Matrix3d& posGeodeticCovariance,
    Eigen::Vector3d& posGeodeticReference,
    const bool is_input_in_degrees = false);

/**
 * @brief Calculate transformation matrix that converts a position deviation
 * from curvilinear (geodetic ECEF) to local navigation frame (Cartesian)
 * representation.
 *
 * @details This function calculates the transformation matrix that converts a
 * curvilinear (geodetic ECEF) position deviation to navigation frame
 * (Cartesian). It is used to, i.e., convert curvilinear errors given in (rad,
 * rad, m) to the local navigation representation in (m, m, m).
 *
 * @see P. D. Groves, Principles of GNSS, Inertial, and Multi-sensor Integrated
 * Navigation Systems, 2013, p. 599.
 *
 * @param[in] posGeodetic Geodetic ECEF position  in (rad, rad, m) or (deg, deg,
 * m) if convert_output_to_degrees is true
 * @param[in] is_input_in_degrees If true, the geodetic input coordinates are in
 * degrees (default: false)
 *
 * @return Transformation matrix that converts local navigation frame
 * (Cartesian) to geodetic ECEF frame
 */
Eigen::Matrix3d calcCartToCurvPosTransfMatrix(
    Eigen::Vector3d& posGeodetic, const bool is_input_in_degrees = false);

/**
 * @brief Calculate transformation matrix that converts a position deviation
 * from local navigation frame (Cartesian) to curvilinear (geodetic ECEF)
 * representation.
 *
 * @details This function calculates the transformation matrix that converts a
 * position deviation local navigation frame (Cartesian) position deviation to
 * curvilinear (geodetic ECEF). It is used to, i.e., convert Cartesian errors
 * given in (m, m, m) to the curvilinear representation (geodetic ECEF) in (rad,
 * rad, m).
 *
 * @see P. D. Groves, Principles of GNSS, Inertial, and Multi-sensor Integrated
 * Navigation Systems, 2013, p. 599.
 *
 * @param[in] posGeodetic Geodetic ECEF position  in (rad, rad, m) or (deg, deg,
 * m) if convert_output_to_degrees is true
 * @param[in] is_input_in_degrees If true, the geodetic input coordinates are in
 * degrees (default: false)
 *
 * @return Transformation matrix that converts local navigation frame
 * (Cartesian) to geodetic ECEF frame
 */
Eigen::Matrix3d calcCurvToCartPosTransfMatrix(
    Eigen::Vector3d& posGeodetic, const bool is_input_in_degrees = false);

/**
 * @brief Calculate the local gravity vector in the local navigation frame
 * at a given latitude and height.
 *
 * @details This function calculates the local gravity vector in the local
 * navigation (n) frame at a given latitude and height. The function uses
 * the European Western European Legal Metrology Cooperation (WELMEC)
 * gravity model. The Earth’s gravity is not constant due to its
 * inhomogeneous mass distribution which makes the gravity vector a function
 * of latitude and height. The local navigation (n) frame is the resolving
 * frame.
 *
 * @see [WELMEC
 * Guide](https://www.welmec.org/welmec/documents/guides/2/WELMEC_Guide_2_v2015.pdf)
 *
 * @param[in] posGeodetic Geodetic ECEF coordinates (latitude, longitude,
 * height) in (rad, rad, m) or (deg, deg, m) if is_input_in_degrees is true
 * @param[in] is_input_in_degrees If true, the geodetic input coordinates
 * are in degrees (default: false)
 *
 * @return Local gravity vector in (m/s², m/s², m/s²)
 */
Eigen::Vector3d getLocalGravityVectorWELMECmodel(
    Eigen::Vector3d& posGeodetic, const bool is_input_in_degrees = false);

/**
 * @brief Calculate the local Earth-rotation vector of the Earth at a given
 * latitude.
 *
 * @details This function calculates the local Earth-rotation vector of the
 * Earth at a given latitude based on input geodetic coordinates. The model uses
 * the angular rate value of Earth from the WGS84 ellipsoid model. The
 * Earth-rotation vector describes the angular velocity of ECEF (e) frame as the
 * object frame which respect to the ECI (i) frame as reference frame. The local
 * navigation (n) frame is the resolving frame.
 *
 * @see P. D. Groves, Principles of GNSS, Inertial, and Multi-sensor Integrated
 * Navigation Systems, 2013, p.67.
 *
 * @param[in] posGeodetic Geodetic ECEF coordinates (latitude, longitude,
 * height) in (rad, rad, m) or (deg, deg, m) if is_input_in_degrees is true
 * @param[in] is_input_in_degrees If true, the geodetic input coordinates are in
 * degrees (default: false)
 *
 * @return Local Earth-rotation vector in (rad/s, rad/s, rad/s)
 */
Eigen::Vector3d getLocalEarthRotationVector(
    Eigen::Vector3d& posGeodetic, const bool is_input_in_degrees = false);

/**
 * @brief Calculte the transport rate for at a given set of geodetic coordinates
 * and NED velocity.
 *
 * @details This function calculates the transport rate for at a given set of
 * geodetic coordinates and NED velocity. The transport rate describes the
 * angular velocity due to rotation of the local navigation frame as the frame
 * origin moves w.r.t. to Earth.
 *
 * @see P. D. Groves, Principles of GNSS, Inertial, and Multi-sensor Integrated
 * Navigation Systems, 2013, p.177.
 *
 * @param[in] posGeodetic Geodetic ECEF coordinates (latitude, longitude,
 * height) in (rad, rad, m) or (deg, deg, m) if is_input_in_degrees is true
 * @param[in] velNed Velocity of body frame w.r.t to navigation frame as
 * reference frame resolved in navigation frame in (m/s, m/s, m/s)
 * @param[in] is_input_in_degrees If true, the geodetic input coordinates are in
 * degrees (default: false)
 *
 * @return Transport rate vector in (rad/s, rad/s, rad/s)
 */
Eigen::Vector3d getTransportRateVector(Eigen::Vector3d& posGeodetic,
                                       const Eigen::Vector3d& velNed,
                                       const bool is_input_in_degrees = false);

/**
 * @brief Converts RPY Euler angles (ZYX order) to a unit quaternion.
 *
 * @details This function converts RPY Euler angles in ZYX order to a unit
 * quaternion. The implementation is based on the MATLAB function euler2q.m from
 * the MSS toolbox of Thor I. Fossen.
 *
 * @see[euler2q.m](https://github.com/cybergalactic/MSS/blob/master/GNC/euler2q.m)
 *
 * @param[in] euler_angles_rpy RPY Euler angles (rad)
 *
 * @return Unit quaternion
 */
Eigen::Quaterniond convertEulerAnglesToQuaternion(
    const Eigen::Vector3d& euler_angles_rpy);

/**
 * @brief Converts a unit quaternion to RPY Euler angles (ZYX order).
 *
 * @details This function converts a unit quaternion to RPY Euler angles in ZYX
 * order. The implementation is based on the MATLAB function q2euler.m from the
 * MSS toolbox of Thor I. Fossen.
 *
 * @see[q2euler.m](https://github.com/cybergalactic/MSS/blob/master/GNC/q2euler.m)
 *
 * @param[in] q Unit quaternion
 * @param[in] convert_output_to_degrees If true, the Euler angles are converted
 * to degrees (default: false)
 *
 * @return RPY Euler angles in (rad) or (deg) if convert_output_to_degrees is
 * true
 */
Eigen::Vector3d convertQuaternionToEulerAngles(
    const Eigen::Quaterniond& q, const bool convert_output_to_degrees = false);

/**
 * @brief Converts RPY Euler angles (ZYX order) to a rotation matrix.
 *
 * @details This function converts RPY Euler angles in ZYX order to a rotation
 * matrix.
 *
 * @param[in] euler_angles_rpy RPY Euler angles (rad)
 *
 * @return Rotation matrix
 */
Eigen::Matrix<double, 3, 3> convertEulerAnglesToRotationMatrix(
    const Eigen::Vector3d& euler_angles_rpy);

/**
 * @brief Converts rotation vector (axis-angle) to quaternion.
 *
 * @details This function converts a rotation vector to a quaternion.
 *
 * @see P. D. Groves, Principles of GNSS, Inertial, and Multi-sensor Integrated
 * Navigation Systems, 2013, p.42-43.
 *
 * @param[in] rv Rotation vector
 *
 * @return Unit quaternion
 */
Eigen::Quaterniond convertRotationVectorToQuaternion(const Eigen::Vector3d& rv);

/**
 * @brief Converts quaternion tp rotation vector (axis-angle).
 *
 * @details This function converts a quaternion to a rotation vector.
 *
 * @see P. D. Groves, Principles of GNSS, Inertial, and Multi-sensor Integrated
 * Navigation Systems, 2013, p.40.
 *
 * @param[in] q Unit quaternion
 *
 * @return Rotation vector
 */
Eigen::Vector3d convertQuaternionToRotationVector(const Eigen::Quaterniond& q);

/**
 * @brief Calculate the skew matrix of a given vector.
 *
 * @details This function calculates the skew matrix of a given vector which
 * is defined as: \f[ \mathbf{V} = \begin{bmatrix}
 * 0 & -v_3 & v_2 \\
 * v_3 & 0 & -v_1 \\
 * -v_2 & v_1 & 0
 * \end{bmatrix}
 * \f]
 *
 * @param[in] v Vector
 *
 * @return 3x3 skew matrix of the given vector
 */
Eigen::Matrix3d calcSkewMatrix3(const Eigen::Vector3d& v);

/**
 * @brief Multiplies two quaternions.
 *
 * @details This function multiplies two quaternions q1 and q2 given as
 * Eigen::Vector4d.
 *
 * @see J. Farrell, Aided Navigation: GPS with High Rate Sensors, in Electronic
 * Engineering, New York: McGraw-Hill, 2008, p. 502.
 *
 * @param[in] q Quaternion q
 * @param[in] r Quaternion r
 *
 * @return Quaternion product as Eigen::Vector4d
 */
Eigen::Vector4d multiplyQuaternions(const Eigen::Vector4d& q,
                                    const Eigen::Vector4d& r);

/**
 * @brief Multiplies two quaternions.
 *
 * @details This function multiplies two quaternions q1 and q2 given as
 * Eigen::Quaterniond.
 *
 * @see J. Farrell, Aided Navigation: GPS with High Rate Sensors, in Electronic
 * Engineering, New York: McGraw-Hill, 2008, p. 502.
 *
 * @param[in] q Quaternion q
 * @param[in] r Quaternion r
 *
 * @return Quaternion product as Eigen::Quaterniond
 */
Eigen::Quaterniond multiplyQuaternions(const Eigen::Quaterniond& q1,
                                       const Eigen::Quaterniond& q2);

/**
 * @brief Integrates a quaternion using the angular rate and the time step.
 *
 * @details This function integrates a quaternion using the angular rate (rad/s)
 * and the time step (s). The Taylor series of the Euler formula is used to
 * approximate the quaternion and to circumvent the singularity (forward
 * integration).
 * @see [Quaternion kinematics for the error-state Kalman filter]
 * (https://arxiv.org/abs/1711.02508)
 *
 * @param[in] q_k Quaternion at time step k
 * @param[in] w Angular rate (rad/s)
 * @param[in] dt Time step (s)
 *
 * @return Integrated quaternion at time step k+1
 */
Eigen::Quaterniond integrateQuaternion(const Eigen::Quaterniond& q,
                                       const Eigen::Vector3d& w,
                                       const double dt);

/**
 * @brief Calculate the modulo of two floating-point numbers.
 *
 * @details Calculate the modulo of two floating-point numbers. As in MATLAB,
 * this function aligns the result with the divisor's sign. The C++ modulo
 * operator does aligns the result with the dividend's sign.
 *
 * @throws std::invalid_argument if `y` is zero.
 *
 * @param[in] x The dividend, a floating-point number
 * @param[in] y The divisor, a floating-point number
 *
 * @return The modulo of `x` and `y`
 */
double modulo(const double x, const double y);

/**
 * @brief Calculate the smallest signed angle between two angles.
 *
 * @details This function calculates the smallest signed angle between two
 * angles.
 * double angle = calculateSmallestSignedAngle(angle) maps an angle in rad
 * to the interval [-pi pi). double angle =
 * calculateSmallestSignedAngle(angle,"deg") maps an angle in deg to the
 * interval [-180 180). The implementation is based on the MATLAB function
 * ssa.m from the MSS toolbox of Thor I. Fossen.
 *
 * @see[ssa.m](https://github.com/cybergalactic/MSS/blob/master/LIBRARY/kinematics/ssa.m)
 *
 * @param[in] angle Angle in rad or deg
 * @param[in] unit Unit of the input angle (default: "rad")
 * @param[in] convert_output_to_degrees If true, the Euler angles are
 * converted to degrees (default: false)
 *
 * @return Smallest signed angle in rad or deg
 */
double calculateSmallestSignedAngle(
    const double angle, const std::string& unit = "rad",
    const bool convert_output_to_degrees = false);

/**
 * @brief Convert geodetic ECEF coordinates from radians to degrees.
 *
 * @param[in] posGeodetic Geodetic ECEF coordinates (latitude, longitude,
 * height) in (rad, rad, m)
 *
 *
 * @return Geodetic ECEF coordinates (latitude, longitude, height) in (deg,
 * deg, m)
 */
Eigen::Vector3d convertPosGeoRadToDeg(const Eigen::Vector3d& posGeodetic);

/**
 * @brief Convert geodetic ECEF coordinates from degrees to radians.
 *
 * @param[in] posGeodetic Geodetic ECEF coordinates (latitude, longitude,
 * height) in (deg, deg, m)
 *
 * @return Geodetic ECEF coordinates (latitude, longitude, height) in (rad, rad,
 * m)
 */
Eigen::Vector3d convertPosGeoDegToRad(const Eigen::Vector3d& posGeodetic);

}  // namespace navigation_utilities
