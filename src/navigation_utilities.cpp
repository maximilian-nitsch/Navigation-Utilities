/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#include "navigation_utilities.hpp"

#include <iostream>
#include <limits>  // For std::numeric_limits

namespace navigation_utilities {

Wgs84Parameters getWGS84Parameters(Eigen::Vector3d& posGeodetic,
                                   const bool is_input_in_degrees) {
  // Convert input geodetic coordinates from degrees to radians if requested
  if (is_input_in_degrees) {
    posGeodetic = convertPosGeoDegToRad(posGeodetic);
  }

  // Create a struct to store the WGS84 parameters
  Wgs84Parameters wgs84Parameters;

  // Extract latitude from the input vector
  const double phi = posGeodetic(0);

  // Semi-major axis of the ellipsoid (m)
  const double a = 6378137.0;

  // Flattening of the ellipsoid
  const double f = 1.0 / 298.257223563;

  // Semi-minor axis of the ellipsoid (m)
  const double b = a * (1 - f);

  // First eccentricity of the ellipsoid
  const double e = std::sqrt(f * (2 - f));

  // Squared first eccentricity of the ellipsoid
  const double e_2 = e * e;

  // Meridian radius of curvature (m)
  const double R_M =
      a * (1 - e_2) / std::pow(1 - e_2 * std::sin(phi) * std::sin(phi), 1.5);

  // Prime vertical radius of curvature (m)
  const double R_N = a / std::sqrt(1 - e_2 * std::sin(phi) * std::sin(phi));

  // Mean radius of curvature (m)
  const double R_0 = std::sqrt(R_M * R_N);

  // Earth angular velocity (rad/s)
  const double omega_ie = 7.2921151467e-5;

  // Assign the parameters to the struct
  wgs84Parameters.a = a;
  wgs84Parameters.b = b;
  wgs84Parameters.f = f;
  wgs84Parameters.e = e;
  wgs84Parameters.R_M = R_M;
  wgs84Parameters.R_N = R_N;
  wgs84Parameters.R_0 = R_0;
  wgs84Parameters.omega_ie = omega_ie;

  return wgs84Parameters;
}

Eigen::Vector3d convertEcefGeodeticToCartesian(Eigen::Vector3d& posGeodetic,
                                               const bool is_input_in_degrees) {
  // Convert input geodetic coordinates from degrees to radians if requested
  if (is_input_in_degrees) {
    posGeodetic = convertPosGeoDegToRad(posGeodetic);
  }

  // Get WGS84 parameters for the given geodetic coordinates
  Wgs84Parameters wgs84Parameters = getWGS84Parameters(posGeodetic);

  // Extract relevant WGS84 parameters from the struct
  const double R_N = wgs84Parameters.R_N;
  const double e = wgs84Parameters.e;

  // Declare variable for the Cartesian ECEF coordinates
  Eigen::Vector3d posCartesian;

  // Extract latitude, longitude, and height from the input vector
  const double phi = posGeodetic(0);
  const double lambda = posGeodetic(1);
  const double h = posGeodetic(2);

  // Calculate the Cartesian ECEF coordinates
  posCartesian(0) = (R_N + h) * std::cos(phi) * std::cos(lambda);
  posCartesian(1) = (R_N + h) * std::cos(phi) * std::sin(lambda);
  posCartesian(2) = (R_N * (1 - e * e) + h) * std::sin(phi);

  // Return the Cartesian ECEF coordinates
  return posCartesian;
}

Eigen::Vector3d convertEcefCartesianToGeodetic(
    const Eigen::Vector3d& posCartesian, const bool convert_output_to_degrees) {
  // WGS84 parameters for zero geodetic coordinates to obtain static values
  Eigen::Vector3d posGeodeticZero = Eigen::Vector3d::Zero();
  Wgs84Parameters wgs84Parameters = getWGS84Parameters(posGeodeticZero);

  // Extract relevant WGS84 parameters from the struct
  double a = wgs84Parameters.a;
  double e = wgs84Parameters.e;

  // Extract x, y, z from the input vector
  const double x = posCartesian(0);
  const double y = posCartesian(1);
  const double z = posCartesian(2);

  const double p = std::sqrt(x * x + y * y);

  double phi = 0.0;
  double h = 0.0;
  double sin_phi = 0.0;
  double R_M = a;  // initial guess

  // Latitude and height need to be calculated iteratively
  for (size_t i = 0; i < 25; i++) {
    sin_phi = z / ((1 - e * e) * R_M + h);
    phi = std::atan((z + e * e * R_M * sin_phi) / p);
    R_M = a / std::sqrt(1 - e * e * std::sin(phi) * std::sin(phi));
    h = p / std::cos(phi) - R_M;
  }

  // Longitude can be calculated explicitly
  double lambda = std::atan2(y, x);

  // Assign the calculated values to the output vector
  Eigen::Vector3d posGeodetic(phi, lambda, h);

  // Convert output geodetic coordinates from radians to degrees if requested
  if (convert_output_to_degrees) {
    posGeodetic = convertPosGeoRadToDeg(posGeodetic);
  }

  // Return the Cartesian ECEF coordinates
  return posGeodetic;
}

Eigen::Vector3d convertGeodeticEcefToNed(Eigen::Vector3d& posGeodetic,
                                         Eigen::Vector3d& posGeodeticReference,
                                         const bool is_input_in_degrees) {
  // Convert input geodetic coordinates from degrees to radians if necessary
  if (is_input_in_degrees) {
    posGeodeticReference = convertPosGeoDegToRad(posGeodeticReference);
    posGeodetic = convertPosGeoDegToRad(posGeodetic);
  }

  // Pre-calculate trigonometric functions
  const double sin_lat = std::sin(posGeodeticReference(0));
  const double cos_lat = std::cos(posGeodeticReference(0));
  const double sin_lon = std::sin(posGeodeticReference(1));
  const double cos_lon = std::cos(posGeodeticReference(1));

  // Rotation matrix from ECEF to NED
  Eigen::Matrix3d C_e_n;
  C_e_n.row(0) << -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat;
  C_e_n.row(1) << -sin_lon, cos_lon, 0;
  C_e_n.row(2) << -cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat;

  // Convert geodetic ECEF coordinates to Cartesian ECEF coordinates
  Eigen::Vector3d posCartesianEcef =
      convertEcefGeodeticToCartesian(posGeodetic);

  // Convert geodetic ECEF reference coordinates to Cartesian ECEF coordinates
  Eigen::Vector3d posReferenceCartesianEcef =
      convertEcefGeodeticToCartesian(posGeodeticReference);

  // Select geodetic ECEF ref. coordinates as origin and rotate ECEF into NED
  Eigen::Vector3d posNed =
      C_e_n * (posCartesianEcef - posReferenceCartesianEcef);

  return posNed;
}

Eigen::Vector3d convertNedToGeodeticEcef(const Eigen::Vector3d& posNed,
                                         Eigen::Vector3d& posGeodeticReference,
                                         const bool is_input_in_degrees,
                                         const bool convert_output_to_degrees) {
  // Convert input geodetic coordinates from degrees to radians if requested
  if (is_input_in_degrees) {
    posGeodeticReference = convertPosGeoDegToRad(posGeodeticReference);
  }

  // Pre-calculate trigonometric functions
  const double sin_lat = std::sin(posGeodeticReference(0));
  const double cos_lat = std::cos(posGeodeticReference(0));
  const double sin_lon = std::sin(posGeodeticReference(1));
  const double cos_lon = std::cos(posGeodeticReference(1));

  // Rotation matrix from NED to ECEF
  Eigen::Matrix3d C_n_e;
  C_n_e.row(0) << -sin_lat * cos_lon, -sin_lon, -cos_lat * cos_lon;
  C_n_e.row(1) << -sin_lat * sin_lon, cos_lon, -cos_lat * sin_lon;
  C_n_e.row(2) << cos_lat, 0, -sin_lat;

  // Convert geodetic ECEF reference coordinates to Cartesian ECEF coordinates
  Eigen::Vector3d posReferenceCartesianEcef =
      convertEcefGeodeticToCartesian(posGeodeticReference);

  // Select geodetic ECEF ref. coordinates as origin and rotate NED into ECEF
  Eigen::Vector3d posCartesianEcef = C_n_e * posNed + posReferenceCartesianEcef;

  // Convert Cartesian ECEF coordinates to geodetic ECEF coordinates
  Eigen::Vector3d posGeodetic =
      convertEcefCartesianToGeodetic(posCartesianEcef);

  // Convert output geodetic coordinates from radians to degrees if requested
  if (convert_output_to_degrees) {
    posGeodetic = convertPosGeoRadToDeg(posGeodetic);
  }

  return posGeodetic;
}

Eigen::Vector3d convertNedToGeodeticEcefStdDev(
    const Eigen::Vector3d& posNedStdDev, Eigen::Vector3d& posGeodeticReference,
    const bool is_input_in_degrees, const bool convert_output_to_degrees) {
  // Convert input geodetic coordinates from degrees to radians if requested
  if (is_input_in_degrees) {
    posGeodeticReference = convertPosGeoDegToRad(posGeodeticReference);
  }
  // Get WGS84 parameters for the given geodetic coordinates
  Wgs84Parameters wgs84Parameters = getWGS84Parameters(posGeodeticReference);

  // Extract relevant WGS84 parameters from the struct
  const double R_M = wgs84Parameters.R_M;
  const double R_N = wgs84Parameters.R_N;

  // Convert geodetic NED standard deviation to geodetic ECEF standard deviation
  Eigen::Vector3d posGeodeticStdDev;
  posGeodeticStdDev(0) = posNedStdDev(0) / R_M;
  posGeodeticStdDev(1) =
      posNedStdDev(1) / (R_N * std::cos(posGeodeticReference(0)));
  posGeodeticStdDev(2) = 0.0;

  // Convert output geodetic coordinates from radians to degrees if requested
  if (convert_output_to_degrees) {
    posGeodeticStdDev = convertPosGeoRadToDeg(posGeodeticStdDev);
  }

  // Height does not change
  posGeodeticStdDev(2) = posNedStdDev(2);

  return posGeodeticStdDev;
}

Eigen::Matrix3d convertNedToGeodeticEcefCovariance(
    const Eigen::Matrix3d& posNedCovariance,
    Eigen::Vector3d& posGeodeticReference, const bool is_input_in_degrees) {
  // Convert input geodetic coordinates from degrees to radians if requested
  if (is_input_in_degrees) {
    posGeodeticReference = convertPosGeoDegToRad(posGeodeticReference);
  }
  // Extract latitude from the input vector
  const double phi = posGeodeticReference(0);

  // Get WGS84 parameters for the given geodetic coordinates
  Wgs84Parameters wgs84Parameters = getWGS84Parameters(posGeodeticReference);

  // Extract relevant WGS84 parameters from the struct
  const double R_M = wgs84Parameters.R_M;
  const double R_N = wgs84Parameters.R_N;

  // Rotation matrix from NED to ECEF
  // Calculate the Jacobian matrix for the transformation
  Eigen::Matrix3d J = Eigen::Matrix3d::Identity();
  J(0, 0) = 1.0 / R_M;
  J(1, 1) = 1.0 / (R_N * std::cos(phi));
  J(2, 2) = -1.0;

  // Transform the covariance matrix from NED to geodetic ECEF
  Eigen::Matrix3d posGeodeticCovariance = J * posNedCovariance * J.transpose();

  return posGeodeticCovariance;
}

Eigen::Matrix3d convertGeodeticEcefToNedCovariance(
    const Eigen::Matrix3d& posGeodeticCovariance,
    Eigen::Vector3d& posGeodeticReference, const bool is_input_in_degrees) {
  // Convert input geodetic coordinates from degrees to radians if requested
  if (is_input_in_degrees) {
    posGeodeticReference = convertPosGeoDegToRad(posGeodeticReference);
  }
  // Extract latitude from the input vector
  const double phi = posGeodeticReference(0);

  // Get WGS84 parameters for the given geodetic coordinates
  Wgs84Parameters wgs84Parameters = getWGS84Parameters(posGeodeticReference);

  // Extract relevant WGS84 parameters from the struct
  const double R_M = wgs84Parameters.R_M;
  const double R_N = wgs84Parameters.R_N;

  // Calculate the Jacobian matrix for the transformation
  Eigen::Matrix3d J = Eigen::Matrix3d::Identity();
  J(0, 0) = R_M;
  J(1, 1) = R_N * std::cos(phi);
  J(2, 2) = -1.0;

  // // // // Transform the covariance matrix from geodetic ECEF to NED
  Eigen::Matrix3d posNedCovariance = J * posGeodeticCovariance * J.transpose();

  return posNedCovariance;
}

Eigen::Matrix3d calcCartToCurvPosTransfMatrix(Eigen::Vector3d& posGeodetic,
                                              const bool is_input_in_degrees) {
  // Convert input geodetic coordinates from degrees to radians if requested
  if (is_input_in_degrees) {
    posGeodetic = convertPosGeoDegToRad(posGeodetic);
  }
  // Extract latitude and height from the input vector
  const double phi = posGeodetic(0);
  const double h = posGeodetic(2);

  // Get WGS84 parameters for the given geodetic coordinates
  Wgs84Parameters wgs84Parameters = getWGS84Parameters(posGeodetic);

  // // Calculate Cartesian-to-curvilinear position change transformation
  Eigen::Matrix3d T_n_e = Eigen::Matrix3d::Identity();
  T_n_e(0, 0) = 1 / (wgs84Parameters.R_M + h);
  T_n_e(1, 1) = 1 / ((wgs84Parameters.R_N + h) * std::cos(phi));
  T_n_e(2, 2) = -1.0;

  return T_n_e;
}

Eigen::Matrix3d calcCurvToCartPosTransfMatrix(Eigen::Vector3d& posGeodetic,
                                              const bool is_input_in_degrees) {
  // Convert input geodetic coordinates from degrees to radians if requested
  if (is_input_in_degrees) {
    posGeodetic = convertPosGeoDegToRad(posGeodetic);
  }
  // Extract latitude and height from the input vector
  const double phi = posGeodetic(0);
  const double h = posGeodetic(2);

  // Get WGS84 parameters for the given geodetic coordinates
  Wgs84Parameters wgs84Parameters = getWGS84Parameters(posGeodetic);

  // // Calculate curvilinear-to-Cartesian position change transformation
  Eigen::Matrix3d T_e_n = Eigen::Matrix3d::Identity();
  T_e_n(0, 0) = wgs84Parameters.R_M + h;
  T_e_n(1, 1) = (wgs84Parameters.R_N + h) * std::cos(phi);
  T_e_n(2, 2) = -1.0;

  return T_e_n;
}

Eigen::Vector3d getLocalGravityVectorWELMECmodel(
    Eigen::Vector3d& posGeodetic, const bool is_input_in_degrees) {
  // Convert input geodetic coordinates from degrees to radians if requested
  if (is_input_in_degrees) {
    posGeodetic = convertPosGeoDegToRad(posGeodetic);
  }

  // Gravity reference values
  const double g_0 = 9.780318;
  const double g_1 = 5.3024e-3;
  const double g_2 = 5.8e-6;
  const double g_3 = 3.085e-6;

  // Extract latitude and height from the input vector
  double phi = posGeodetic(0);
  double h = posGeodetic(2);

  // Local gravity vector according WELMEC model
  Eigen::Vector3d g_n(0.0, 0.0,
                      g_0 * (1 + g_1 * std::pow(std::sin(phi), 2) -
                             g_2 * std::pow(std::sin(2 * phi), 2)) -
                          g_3 * h);

  return g_n;
}

Eigen::Vector3d getLocalEarthRotationVector(Eigen::Vector3d& posGeodetic,
                                            const bool is_input_in_degrees) {
  // Convert input geodetic coordinates from degrees to radians if requested
  if (is_input_in_degrees) {
    posGeodetic = convertPosGeoDegToRad(posGeodetic);
  }

  // Get WGS84 parameters for the given geodetic coordinates
  Wgs84Parameters wgs84Parameters = getWGS84Parameters(posGeodetic);

  // Extract latitude from the input vector
  const double phi = posGeodetic(0);

  // Local Earth-rotation vector resolved in the local navigation frame
  Eigen::Vector3d w_ie_n(wgs84Parameters.omega_ie * std::cos(phi), 0.0,
                         -wgs84Parameters.omega_ie * std::sin(phi));

  return w_ie_n;
}

Eigen::Vector3d getTransportRateVector(
    Eigen::Vector3d& posGeodetic, const Eigen::Vector3d& velNed,
    const bool is_input_in_degrees) {  // Convert input geodetic coordinates
                                       // from degrees to radians if requested
  if (is_input_in_degrees) {
    posGeodetic = convertPosGeoDegToRad(posGeodetic);
  }

  // Get WGS84 parameters for the given geodetic coordinates
  Wgs84Parameters wgs84Parameters = getWGS84Parameters(posGeodetic);

  // Extract latitude from the input vector
  const double phi = posGeodetic(0);
  const double h = posGeodetic(2);

  // Calculate transport rate vector
  Eigen::Vector3d w_en_n(
      velNed(1) / (wgs84Parameters.R_N + h),
      -velNed(0) / (wgs84Parameters.R_M + h),
      -velNed(1) * std::tan(phi) / (wgs84Parameters.R_N + h));

  return w_en_n;
}

Eigen::Quaterniond convertEulerAnglesToQuaternion(
    const Eigen::Vector3d& euler_angles_rpy) {
  // Get the Euler angles from the input vector
  double roll = euler_angles_rpy[0];
  double pitch = euler_angles_rpy[1];
  double yaw = euler_angles_rpy[2];

  // Precompute trigonometric functions
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);

  // Calculate the quaternion
  Eigen::Quaterniond q;
  q.w() = cy * cp * cr + sy * sp * sr;
  q.x() = cy * cp * sr - sy * sp * cr;
  q.y() = sy * cp * sr + cy * sp * cr;
  q.z() = sy * cp * cr - cy * sp * sr;

  return q;
}

Eigen::Vector3d convertQuaternionToEulerAngles(
    const Eigen::Quaterniond& q, const bool convert_output_to_degrees) {
  // Ensure that the quaternion is normalized
  Eigen::Quaterniond quat = q.normalized();

  // Calculate the rotation matrix
  Eigen::Matrix3d C = quat.toRotationMatrix();

  // Calculate roll angle from rotation matrix
  double roll = atan2(C(2, 1), C(2, 2));

  // Get the sign of the pitch angle from the rotation matrix
  if (abs(C(2, 0)) > 1)
    C(2, 0) = C(2, 0) / abs(C(2, 0));

  // Calculate pitch angle from rotation matrix
  double pitch = -asin(C(2, 0));

  // Calculate yaw angle from rotation matrix
  double yaw = atan2(C(1, 0), C(0, 0));

  // Assemble the Euler angles in a vector
  Eigen::Vector3d eulerAnglesRpy = Eigen::Vector3d(roll, pitch, yaw);

  // Convert Euler angles from radians to degrees if requested
  if (convert_output_to_degrees) {
    eulerAnglesRpy *= 180.0 / M_PI;
  }

  return eulerAnglesRpy;
}

Eigen::Quaterniond integrateQuaternion(const Eigen::Quaterniond& q_k,
                                       const Eigen::Vector3d& w,
                                       const double dt) {
  // Change of attitude during dt given as orientation vector (ov)
  Eigen::Vector3d delOv = w * dt;

  // Norm of incremental orientation vector
  double delOvAbs = delOv.norm();

  const double angleThreshold = 5.0 * M_PI / 180.0;

  Eigen::Vector4d r_k;

  if (delOvAbs < angleThreshold) {
    // Small angle change: approximated with Taylor series in order to
    // circumvent the singularity
    r_k << 1 - (1.0 / 8) * delOvAbs * delOvAbs +
               (1.0 / 384) * std::pow(delOvAbs, 4) -
               (1.0 / 46080) * std::pow(delOvAbs, 6),
        delOv * (0.5 - (1.0 / 48) * delOvAbs * delOvAbs +
                 (1.0 / 3840) * std::pow(delOvAbs, 4) -
                 (1.0 / 645120) * std::pow(delOvAbs, 6));
  } else {
    // Large angle change: Sine/cosine-based calculation
    r_k << std::cos(delOvAbs / 2.0),
        delOv * (std::sin(delOvAbs / 2.0) / delOvAbs);
  }

  // Add change of attitude to the last quaternion for integration
  Eigen::Quaterniond q_k_1 =
      q_k * Eigen::Quaterniond(r_k(0), r_k(1), r_k(2), r_k(3));

  q_k_1.normalize();

  return q_k_1;
}

Eigen::Matrix<double, 3, 3> convertEulerAnglesToRotationMatrix(
    const Eigen::Vector3d& euler_angles_rpy) {
  // Get the Euler angles from the input vector
  double roll = euler_angles_rpy[0];
  double pitch = euler_angles_rpy[1];
  double yaw = euler_angles_rpy[2];

  // Precompute trigonometric functions
  double cr = cos(roll);
  double sr = sin(roll);
  double cp = cos(pitch);
  double sp = sin(pitch);
  double cy = cos(yaw);
  double sy = sin(yaw);

  // Calculate the rotation matrix
  Eigen::Matrix<double, 3, 3> C;
  C << cp * cy, -cr * sy + sr * sp * cy, sr * sy + cr * sp * cy, cp * sy,
      cr * cy + sr * sp * sy, -sr * cy + cr * sp * sy, -sp, sr * cp, cr * cp;

  return C;
}

Eigen::Quaterniond convertRotationVectorToQuaternion(
    const Eigen::Vector3d& rv) {
  // Get the machine epsilon for double
  double eps = std::numeric_limits<double>::epsilon();

  // Define the quaternion
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

  // Calculate norm (length) of rotation vector
  double rvn = rv.norm();

  // Construct quaternion from rotation vector
  if (rvn > 100 * eps) {
    double a = rvn / 2;
    double s = std::sin(a) / rvn;

    q.w() = std::cos(a);
    q.x() = rv.x() * s;
    q.y() = rv.y() * s;
    q.z() = rv.z() * s;
  }

  // Ensure quaternion normalization
  q.normalize();

  return q;
}

Eigen::Vector3d convertQuaternionToRotationVector(const Eigen::Quaterniond& q) {
  // Get the machine epsilon for double
  double eps = std::numeric_limits<double>::epsilon();

  // Define rotation vector
  Eigen::Vector3d rv = Eigen::Vector3d::Identity() * eps;

  // Ensure unit quaternion
  Eigen::Quaterniond qn = q.normalized();

  double theta = std::acos(qn.w());
  double s = std::sin(theta / 2);

  // Construct rotation vector from quaternion
  if (std::fabs(s) > 100 * eps) {
    rv.x() = qn.x() / s;
    rv.y() = qn.y() / s;
    rv.z() = qn.z() / s;
    rv = theta * rv;
  }

  return rv;
}

Eigen::Matrix3d calcSkewMatrix3(const Eigen::Vector3d& v) {
  // Declare the skew-symmetric matrix
  Eigen::Matrix3d V;
  V << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;

  return V;
}

Eigen::Vector4d multiplyQuaternions(const Eigen::Vector4d& q,
                                    const Eigen::Vector4d& r) {
  // Construct the matrix Q based on quaternion q
  Eigen::Matrix4d Q;
  Q << q(0), -q(1), -q(2), -q(3), q(1), q(0), -q(3), q(2), q(2), q(3), q(0),
      -q(1), q(3), -q(2), q(1), q(0);

  // Perform quaternion multiplication
  Eigen::Vector4d qm = Q * r;

  // Normalize the resulting quaternion
  double norm_qm = qm.norm();

  return qm / norm_qm;
}

Eigen::Quaterniond multiplyQuaternions(const Eigen::Quaterniond& q,
                                       const Eigen::Quaterniond& r) {
  // Construct the matrix Q based on quaternion q
  Eigen::Matrix4d Q;
  Q << q.w(), -q.x(), -q.y(), -q.z(), q.x(), q.w(), -q.z(), q.y(), q.y(), q.z(),
      q.w(), -q.x(), q.z(), -q.y(), q.x(), q.w();

  // Perform quaternion multiplication
  Eigen::Vector4d qmv = Q * Eigen::Vector4d(r.w(), r.x(), r.y(), r.z());

  // Assign quaternion in vector form to Quaternion class
  Eigen::Quaterniond qm(qmv(0), qmv(1), qmv(2), qmv(3));

  // Normalize the resulting quaternion
  qm.normalize();

  return qm;
}

double modulo(const double x, const double y) {
  if (y == 0.0) {
    throw std::invalid_argument("modulo(): the divisor 'y' must not be zero.");
  }

  return x - std::floor(x / y) * y;
}

double calculateSmallestSignedAngle(const double angle, const std::string& unit,
                                    const bool convert_output_to_degrees) {
  // Define smallest signed angle
  double ssa;

  if (unit == "rad") {
    // Map the angle to [-pi, pi)
    ssa = modulo(angle + M_PI, 2 * M_PI) - M_PI;
  } else if (unit == "deg") {
    // Map the angle to [-180, 180)
    ssa = modulo(angle + 180.0, 360.0) - 180.0;
  } else {
    std::cerr << "calculateSmallestSignedAngle(): unknown unit! Use 'rad' for "
                 "radians or 'deg' for degrees.\n";
    ssa = angle;  // Unchanged angle if unit is unknown
  }
  // Convert the angle to degrees if requested
  if (convert_output_to_degrees) {
    ssa *= 180.0 / M_PI;
  }

  return ssa;
}

Eigen::Vector3d convertPosGeoRadToDeg(const Eigen::Vector3d& posGeodetic) {
  Eigen::Vector3d posGeodeticDeg(posGeodetic);
  posGeodeticDeg(0) = posGeodetic(0) * 180.0 / M_PI;
  posGeodeticDeg(1) = posGeodetic(1) * 180.0 / M_PI;

  return posGeodeticDeg;
}

Eigen::Vector3d convertPosGeoDegToRad(const Eigen::Vector3d& posGeodetic) {
  Eigen::Vector3d posGeodeticRad(posGeodetic);
  posGeodeticRad(0) = posGeodetic(0) * M_PI / 180.0;
  posGeodeticRad(1) = posGeodetic(1) * M_PI / 180.0;

  return posGeodeticRad;
}

}  // namespace navigation_utilities