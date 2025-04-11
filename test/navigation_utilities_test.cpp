/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#include "navigation_utilities.hpp"

#include "gtest/gtest.h"

namespace navigation_utilities {

TEST(NavigationUtilitiesTest, WGS84ParametersTest) {
  // Geodetic cooordinates for the test
  Eigen::Vector3d posGeodetic(0.5934119541, -2.0478571082,
                              251.702);  // (rad, rad, m)

  // Get the WGS84 parameters for the given geodetic coordinates
  Wgs84Parameters wgs84Parameters = getWGS84Parameters(posGeodetic);

  // Check the values of the WGS84 parameters
  EXPECT_NEAR(wgs84Parameters.a, 6378137.0, 1e-9);
  EXPECT_NEAR(wgs84Parameters.b, 6356752.31424518, 1e-9);
  EXPECT_NEAR(wgs84Parameters.f, 0.00335281066474748, 1e-9);
  EXPECT_NEAR(wgs84Parameters.e, 0.0818191908426215, 1e-9);
  EXPECT_NEAR(wgs84Parameters.R_M, 6355384.5711715, 1e-6);
  EXPECT_NEAR(wgs84Parameters.R_N, 6384823.20998238, 1e-6);
  EXPECT_NEAR(wgs84Parameters.R_0, 6370086.88468059, 1e-6);
  EXPECT_NEAR(wgs84Parameters.omega_ie, 7.2921151467e-5, 1e-9);
}

TEST(NavigationUtilitiesTest, ConvertEcefGeodeticToCartesianTest) {
  // Geodetic cooordinates for the test
  Eigen::Vector3d posGeodetic(0.5934119541, -2.0478571082,
                              251.702);  // (rad, rad, m)

  // Convert the geodetic coordinates to Cartesian coordinates
  Eigen::Vector3d posCartesian = convertEcefGeodeticToCartesian(posGeodetic);

  // Expected Cartesian coordinates
  Eigen::Vector3d posCartesianExpected(-2430601.82757189, -4702442.70320792,
                                       3546587.35812853);  // (m, m, m)

  // Check if the Cartesian coordinates are calculated correctly
  EXPECT_NEAR(posCartesian(0), posCartesianExpected(0), 1e-6);
  EXPECT_NEAR(posCartesian(1), posCartesianExpected(1), 1e-6);
  EXPECT_NEAR(posCartesian(2), posCartesianExpected(2), 1e-6);
}

TEST(NavigationUtilitiesTest, ConvertEcefCartesianToGeodeticTest) {
  // Cartesian coordinates for the test
  Eigen::Vector3d posCartesian(-2430601.82757189, -4702442.70320792,
                               3546587.35812853);  // (m, m, m)

  // Convert the Cartesian coordinates to geodetic coordinates
  Eigen::Vector3d posGeodetic = convertEcefCartesianToGeodetic(posCartesian);

  // Expected geodetic coordinates
  Eigen::Vector3d posGeodeticExpected(0.5934119541, -2.0478571082,
                                      251.702);  // (rad, rad, m)

  // Check if the geodetic coordinates are calculated correctly
  EXPECT_NEAR(posGeodetic(0), posGeodeticExpected(0), 1e-6);
  EXPECT_NEAR(posGeodetic(1), posGeodeticExpected(1), 1e-6);
  EXPECT_NEAR(posGeodetic(2), posGeodeticExpected(2), 1e-6);
}

TEST(NavigationUtilitiesTest, ConvertGeodeticEcefToNedTest) {
  // Geodetic coordinates for the test
  Eigen::Vector3d posGeodetic(0.591392563455162, -2.04329741326952,
                              -200.867236523889);  // (rad, rad, m)

  // Geodetic cooordinates of the local NED origin for the test
  Eigen::Vector3d posGeodeticReference(0.5912037433, -2.0432030931,
                                       120.0);  // (rad, rad, m)

  // Convert the local NED coordinates to geodetic ECEF coordinates
  Eigen::Vector3d posNed =
      convertGeodeticEcefToNed(posGeodetic, posGeodeticReference);

  // Expected NED coordinates (MATLAB: lla2ned(posGeodetic,
  // posGeodeticReference, 'ellipsoid'))
  Eigen::Vector3d posNedExpected(1199.97626075789, -499.921177702181,
                                 321.000099891573);  // (m, m, m)

  // Check if the geodetic coordinates are calculated correctly
  EXPECT_NEAR(posNed(0), posNedExpected(0), 1e-6);
  EXPECT_NEAR(posNed(1), posNedExpected(1), 1e-6);
  EXPECT_NEAR(posNed(2), posNedExpected(2), 1e-6);
}

TEST(NavigationUtilitiesTest, ConvertNedToGeodeticEcefTest) {
  // NED coordinates for the test
  Eigen::Vector3d posNed(1200.0, -500.0, 321.0);  // (m, m, m)

  // Geodetic cooordinates of the local NED origin for the test
  Eigen::Vector3d posGeodeticReference(0.5912037433, -2.0432030931,
                                       120.0);  // (rad, rad, m)

  // Convert the local NED coordinates to geodetic ECEF coordinates
  Eigen::Vector3d posGeodetic =
      convertNedToGeodeticEcef(posNed, posGeodeticReference);

  // Expected geodetic coordinates
  Eigen::Vector3d posGeodeticExpected(0.591392567189991, -2.04329742814116,
                                      -200.867125976831);  // (rad, rad, m)

  // Check if the geodetic coordinates are calculated correctly
  EXPECT_NEAR(posGeodetic(0), posGeodeticExpected(0), 1e-6);
  EXPECT_NEAR(posGeodetic(1), posGeodeticExpected(1), 1e-6);
  EXPECT_NEAR(posGeodetic(2), posGeodeticExpected(2), 1e-6);

  // Convert the local NED coordinates to geodetic ECEF coordinates in degrees
  Eigen::Vector3d posGeodeticDegree =
      convertNedToGeodeticEcef(posNed, posGeodeticReference, false, true);

  // Check if the geodetic coordinates in degrees are calculated correctly
  EXPECT_NEAR(posGeodeticDegree(0), posGeodeticExpected(0) * 180 / M_PI, 1e-6);
  EXPECT_NEAR(posGeodeticDegree(1), posGeodeticExpected(1) * 180 / M_PI, 1e-6);
  EXPECT_NEAR(posGeodeticDegree(2), posGeodeticExpected(2), 1e-6);
}

TEST(NavigationUtilitiesTest, GetLocalGravityVectorWELMECmodelTest) {
  // Geodetic cooordinates for the test
  Eigen::Vector3d posGeodetic(0.5934119541, -2.0478571082,
                              251.702);  // (rad, rad, m)

  // Calculate the gravity vector for the given geodetic coordinates
  Eigen::Vector3d gravity = getLocalGravityVectorWELMECmodel(posGeodetic);

  // Expected gravity vector
  Eigen::Vector3d gravityExpected = Eigen::Vector3d(0.0, 0.0, 9.79570892202643);

  // Check if the gravity vector is calculated correctly
  EXPECT_NEAR(gravity(0), gravityExpected(0), 1e-6);
  EXPECT_NEAR(gravity(1), gravityExpected(1), 1e-6);
  EXPECT_NEAR(gravity(2), gravityExpected(2), 1e-6);
}

TEST(NavigationUtilitiesTest, GetLocalEarthRotationVectorTest) {
  // Geodetic cooordinates for the test
  Eigen::Vector3d posGeodetic(0.5934119541, -2.0478571082,
                              251.702);  // (rad, rad, m)

  // Calculate the Earth rotation vector for the given geodetic coordinates
  Eigen::Vector3d earthRotationVector =
      getLocalEarthRotationVector(posGeodetic);

  // Expected Earth rotation vector
  Eigen::Vector3d earthRotationVectorExpected(6.04543728405012e-05, 0.0,
                                              -4.07769901020682e-05);

  // Check if the Earth rotation vector is calculated correctly
  EXPECT_NEAR(earthRotationVector(0), earthRotationVectorExpected(0), 1e-6);
  EXPECT_NEAR(earthRotationVector(1), earthRotationVectorExpected(1), 1e-6);
  EXPECT_NEAR(earthRotationVector(2), earthRotationVectorExpected(2), 1e-6);
}

TEST(NavigationUtilitiesTest, GetTransportRateVectorTest) {
  // Geodetic cooordinates for the test
  Eigen::Vector3d posGeodetic(0.5934119541, -2.0478571082,
                              251.702);   // (rad, rad, m)
  Eigen::Vector3d velNed(1.0, 2.0, 3.0);  // (m/s, m/s, m/s)

  // Calculate the transport rate vector for the given geodetic coordinates
  // and NED velocity
  Eigen::Vector3d transportRateVector =
      getTransportRateVector(posGeodetic, velNed);

  // Eigen::Vector3d transportRateVectorExpected(
  //     0.000136321376486347, 0.000272642752972694, 0.000408964129459041);

  Eigen::Vector3d transportRateVectorExpected(
      0.313255164814033e-6, -0.157353122363650e-6, -0.211293280450442e-6);

  // Check if the transport rate vector is calculated correctly
  EXPECT_NEAR(transportRateVector(0), transportRateVectorExpected(0), 1e-6);
  EXPECT_NEAR(transportRateVector(1), transportRateVectorExpected(1), 1e-6);
  EXPECT_NEAR(transportRateVector(2), transportRateVectorExpected(2), 1e-6);
}

TEST(NavigationUtilitiesTest, CalculateSmallestSignedAngleTest) {
  // Define test angles
  double test_angle_1 = 10 - (-30);
  double test_angle_2 = -30 - (+10);
  double test_angle_3 = -170 - (+170);
  double test_angle_4 = 170 - (-170);
  double test_angle_5 = -170.1 - (+170.1);
  double test_angle_6 = 170.1 - (-170.1);
  double test_angle_7 = 5.2 - (-5.1);

  // Check if calculateSmallestSignedAngle function works correctly
  EXPECT_NEAR(calculateSmallestSignedAngle(test_angle_1, "deg"), 40.0, 1e-6);
  EXPECT_NEAR(calculateSmallestSignedAngle(test_angle_2, "deg"), -40.0, 1e-6);
  EXPECT_NEAR(calculateSmallestSignedAngle(test_angle_3, "deg"), 20.0, 1e-6);
  EXPECT_NEAR(calculateSmallestSignedAngle(test_angle_4, "deg"), -20.0, 1e-6);
  EXPECT_NEAR(calculateSmallestSignedAngle(test_angle_5, "deg"), 19.8, 1e-6);
  EXPECT_NEAR(calculateSmallestSignedAngle(test_angle_6, "deg"), -19.8, 1e-6);
  EXPECT_NEAR(calculateSmallestSignedAngle(test_angle_7, "deg"), 10.3, 1e-6);
}

TEST(NavigationUtilitiesTest, ConvertEulerAnglesToQuaternionTest) {
  // Euler angles
  Eigen::Vector3d euler_angles = Eigen::Vector3d(45, 25, -60) * M_PI / 180;

  // Expected result
  Eigen::Quaterniond expected_quaternion(0.739723578753529, 0.423539813707324,
                                         -0.0136321376486347,
                                         -0.52272097534341);

  // Call the function
  Eigen::Quaterniond result_quaternion =
      convertEulerAnglesToQuaternion(euler_angles);

  // Check if the result is close to the expected value
  EXPECT_NEAR(result_quaternion.coeffs()[0], expected_quaternion.coeffs()[0],
              1e-9);
  EXPECT_NEAR(result_quaternion.coeffs()[1], expected_quaternion.coeffs()[1],
              1e-9);
  EXPECT_NEAR(result_quaternion.coeffs()[2], expected_quaternion.coeffs()[2],
              1e-9);
  EXPECT_NEAR(result_quaternion.coeffs()[3], expected_quaternion.coeffs()[3],
              1e-9);

  // Euler angles
  euler_angles = Eigen::Vector3d(-15, 5, 90) * M_PI / 180;

  // Expected result
  expected_quaternion =
      Eigen::Quaterniond(0.696364240320019, -0.122787803968973,
                         -0.0616284167162193, 0.704416026402759);

  // Call the function
  result_quaternion = convertEulerAnglesToQuaternion(euler_angles);

  // Check if the result is close to the expected value
  EXPECT_NEAR(result_quaternion.coeffs()[0], expected_quaternion.coeffs()[0],
              1e-9);
  EXPECT_NEAR(result_quaternion.coeffs()[1], expected_quaternion.coeffs()[1],
              1e-9);
  EXPECT_NEAR(result_quaternion.coeffs()[2], expected_quaternion.coeffs()[2],
              1e-9);
  EXPECT_NEAR(result_quaternion.coeffs()[3], expected_quaternion.coeffs()[3],
              1e-9);
}

TEST(NavigationUtilitiesTest, ConvertQuaternionToEulerAnglesTest) {
  // Quaternion, corresponding to roll = 45 deg, pitch = 25 deg, yaw = -60 deg
  Eigen::Quaterniond quat(0.739723578753529, 0.423539813707324,
                          -0.0136321376486347, -0.52272097534341);

  // Expected euler angles in degrees
  Eigen::Vector3d expectedEulerAnglesInDegree(45, 25, -60);

  // Call the function
  Eigen::Vector3d eulerAnglesInDegree =
      convertQuaternionToEulerAngles(quat, true);

  // Check if the result is close to the expected value
  EXPECT_NEAR(eulerAnglesInDegree(0), expectedEulerAnglesInDegree(0), 1e-9);
  EXPECT_NEAR(eulerAnglesInDegree(1), expectedEulerAnglesInDegree(1), 1e-9);
  EXPECT_NEAR(eulerAnglesInDegree(2), expectedEulerAnglesInDegree(2), 1e-9);

  // Quaternion, corresponding to roll = -15 deg, pitch = 5 deg, yaw = 180 deg
  quat = Eigen::Quaterniond(0.696364240320019, -0.122787803968973,
                            -0.0616284167162193, 0.704416026402759);

  // Expected result
  expectedEulerAnglesInDegree = Eigen::Vector3d(-15, 5, 90);

  // Call the function
  eulerAnglesInDegree = convertQuaternionToEulerAngles(quat, true);

  // Check if the result is close to the expected value
  EXPECT_NEAR(eulerAnglesInDegree(0), expectedEulerAnglesInDegree(0), 1e-9);
  EXPECT_NEAR(eulerAnglesInDegree(1), expectedEulerAnglesInDegree(1), 1e-9);
  EXPECT_NEAR(eulerAnglesInDegree(2), expectedEulerAnglesInDegree(2), 1e-9);
}

TEST(NavigationUtilitiesTest, ConvertEulerAnglesToRotationMatrixTest) {
  // Euler angles
  Eigen::Vector3d euler_angles = Eigen::Vector3d(45, 25, -60) * M_PI / 180;

  // Expected result
  Eigen::Matrix3d expected_rotation_matrix;
  expected_rotation_matrix << 0.453153893518325, 0.761790555060855,
      -0.462954316330735, -0.784885567221396, 0.094753616281599,
      -0.612353164904949, -0.422618261740699, 0.640856382055788,
      0.640856382055789;

  // Call the function
  Eigen::Matrix3d result_rotation_matrix =
      convertEulerAnglesToRotationMatrix(euler_angles);

  // Check if the result is close to the expected value
  for (size_t i = 0; i < 2; i++) {
    for (size_t j = 0; j < 2; j++) {
      EXPECT_NEAR(result_rotation_matrix(i, j), expected_rotation_matrix(i, j),
                  1e-9);
    }
  }

  // Euler angles
  euler_angles = Eigen::Vector3d(-15, 5, 90) * M_PI / 180;

  // Expected result
  expected_rotation_matrix << 6.0999332417281e-17, -0.965925826289068,
      -0.258819045102521, 0.996194698091746, -0.0225575661131498,
      0.0841859828293692, -0.0871557427476582, -0.2578341604963,
      0.962250186899058;

  // Call the function
  result_rotation_matrix = convertEulerAnglesToRotationMatrix(euler_angles);

  // Check if the result is close to the expected value
  for (size_t i = 0; i < 2; i++) {
    for (size_t j = 0; j < 2; j++) {
      EXPECT_NEAR(result_rotation_matrix(i, j), expected_rotation_matrix(i, j),
                  1e-9);
    }
  }
}

TEST(NavigationUtilitiesTest, convertRotationVectorToQuaternionTest) {
  // Test rotation vector
  Eigen::Vector3d rv = Eigen::Vector3d::Zero();

  // Convert rotation vector to unit quaternion
  Eigen::Quaterniond q = convertRotationVectorToQuaternion(rv);
  Eigen::Quaterniond q_expected = Eigen::Quaterniond::Identity();

  // Check if the unit quaternion is calculated correctly
  EXPECT_NEAR(q.w(), q_expected.w(), 1e-9);
  EXPECT_NEAR(q.x(), q_expected.x(), 1e-9);
  EXPECT_NEAR(q.y(), q_expected.y(), 1e-9);
  EXPECT_NEAR(q.z(), q_expected.z(), 1e-9);

  // Second test rotation vector
  rv << 1.402810153933952, 0.581062991194132, -1.402810153933952;

  // Convert rotation vector to unit quaternion
  q = convertRotationVectorToQuaternion(rv);
  q_expected.w() = 0.511721860575176;
  q_expected.x() = 0.583018563452643;
  q_expected.y() = 0.241494196097364;
  q_expected.z() = -0.583018563452643;

  // Check if the unit quaternion is calculated correctly
  EXPECT_NEAR(q.w(), q_expected.w(), 1e-9);
  EXPECT_NEAR(q.x(), q_expected.x(), 1e-9);
  EXPECT_NEAR(q.y(), q_expected.y(), 1e-9);
  EXPECT_NEAR(q.z(), q_expected.z(), 1e-9);
}

TEST(NavigationUtilitiesTest, convertQuaternionToRotationVectorTest) {
  // Test unit quaternion
  Eigen::Quaterniond q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

  // Convert unit quaternion to rotation vector
  Eigen::Vector3d rv = convertQuaternionToRotationVector(q);
  Eigen::Vector3d rv_expected = Eigen::Vector3d::Zero();

  // Check if the rotation vector is calculated correctly
  EXPECT_NEAR(rv.x(), rv_expected.x(), 1e-9);
  EXPECT_NEAR(rv.y(), rv_expected.y(), 1e-9);
  EXPECT_NEAR(rv.z(), rv_expected.z(), 1e-9);

  // Second test unit quaternion
  q.w() = 0.270598050073099;
  q.x() = 0.653281482438188;
  q.y() = 0.270598050073099;
  q.z() = -0.653281482438188;

  // Convert rotation vector to unit quaternion
  rv = convertQuaternionToRotationVector(q);
  rv_expected << 1.402810153933952, 0.581062991194132, -1.402810153933952;

  // Check if the rotation vector is calculated correctly
  EXPECT_NEAR(rv.x(), rv_expected.x(), 1e-9);
  EXPECT_NEAR(rv.y(), rv_expected.y(), 1e-9);
  EXPECT_NEAR(rv.z(), rv_expected.z(), 1e-9);
}

TEST(NavigationUtilitiesTest, SkewMatrix3Test) {
  // Input vector
  Eigen::Vector3d v(1.0, 2.0, 3.0);

  // Expected result
  Eigen::Matrix3d expectedV;
  expectedV << 0, -3, 2, 3, 0, -1, -2, 1, 0;

  // Call the function
  Eigen::Matrix3d resultV = calcSkewMatrix3(v);

  // Check if the result is close to the expected value
  EXPECT_TRUE(resultV.isApprox(expectedV, 1e-9));
}

TEST(NavigationUtilitiesTest, ConvertNedToGeodeticEcefStdDevTest) {
  // Geodetic cooordinates for the test
  Eigen::Vector3d posGeodetic(0.5934119541, -2.0478571082,
                              251.702);  // (rad, rad, m)

  // Standard deviations for the test
  Eigen::Vector3d posNedStdDev =
      Eigen::Vector3d(0.006, 0.006, 0.01);  // (m, m, m)

  // Transform the NED  standard deviations to geodetic ECEF standard deviations
  Eigen::Vector3d posGeodeticCovariance =
      convertNedToGeodeticEcefStdDev(posNedStdDev, posGeodetic);

  // Expected result
  Eigen::Vector3d posGeodeticStdDevExpected(5.41e-8, 6.49e-8, 0.01);

  // Check if the result is close to the expected value
  EXPECT_NEAR(posGeodeticCovariance(0), posGeodeticStdDevExpected(0), 1e-6);
  EXPECT_NEAR(posGeodeticCovariance(1), posGeodeticStdDevExpected(1), 1e-6);
  EXPECT_NEAR(posGeodeticCovariance(2), posGeodeticStdDevExpected(2), 1e-6);
}

TEST(NavigationUtilitiesTest, QuaternionIntegrationTest1) {
  // Initial quaternion
  Eigen::Quaterniond q_k(1.0, 0.0, 0.0, 0.0);

  // Angular rate
  Eigen::Vector3d w(0.1, 0.2, 0.3);

  // Time step
  double dt = 0.01;

  // Expected result from MATLAB simulation
  Eigen::Quaterniond expected_q_k_1(0.99999825000051, 0.000499999708333384,
                                    0.000999999416666769, 0.00149999912500015);
  // Call the function
  Eigen::Quaterniond result_q_k_1 = integrateQuaternion(q_k, w, dt);

  // Check if the result is close to the expected value
  EXPECT_TRUE(result_q_k_1.isApprox(expected_q_k_1, 1e-9));
}

TEST(NavigationUtilitiesTest, QuaternionIntegrationTest2) {
  // Initial quaternion
  Eigen::Quaterniond q_k(0.7071, 0.7071, 0.0, 0.0);

  // Angular rate
  Eigen::Vector3d w(0.1, 0.2, 0.3);

  // Time step
  double dt = 0.01;

  // Expected result from MATLAB simulation
  Eigen::Quaterniond expected_q_k_1(0.706745212781598, 0.707452312369123,
                                    -0.000353549793762536, 0.00176774896881268);

  // Call the function
  Eigen::Quaterniond result_q_k_1 = integrateQuaternion(q_k, w, dt);

  // Check if the result is close to the expected value
  EXPECT_TRUE(result_q_k_1.isApprox(expected_q_k_1, 1e-8));
}

TEST(NavigationUtilitiesTest, ConvertGeodeticEcefToNedCovarianceTest) {
  // Geodetic position covariance for the test
  Eigen::Matrix3d posGeodeticCovariance = Eigen::Matrix3d::Identity();
  posGeodeticCovariance(0, 0) = std::pow(0.1 * M_PI / 180.0, 2);
  posGeodeticCovariance(1, 1) = std::pow(0.1 * M_PI / 180.0, 2);
  posGeodeticCovariance(2, 2) = std::pow(1.0, 2);

  // Geodetic cooordinates of the local NED origin for the test
  Eigen::Vector3d posGeodeticReference(0.5912037433, -2.0432030931,
                                       120.0);  // (rad, rad, m)

  Eigen::Matrix3d posNedCovariance = convertGeodeticEcefToNedCovariance(
      posGeodeticCovariance, posGeodeticReference);

  std::cout << "posNedCovariance:\n " << posNedCovariance << std::endl
            << std::endl;

  std::cout << posNedCovariance.diagonal().array().sqrt() << std::endl;

  // 1 deg lat/lon standard deviation correspontd to approx. 111.32 km
  EXPECT_NEAR(std::sqrt(posNedCovariance(0, 0)), 11125.86, 100);
  EXPECT_NEAR(std::sqrt(posNedCovariance(1, 1)), 9394.79, 100);
  EXPECT_NEAR(std::sqrt(posNedCovariance(2, 2)), 1.0, 1);
}

}  // namespace navigation_utilities
