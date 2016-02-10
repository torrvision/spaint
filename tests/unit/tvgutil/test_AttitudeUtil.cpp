#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <ITMLib/Utils/ITMMath.h>

#include <tvgutil/AttitudeUtil.h>
using namespace tvgutil;

//TO REMOVE
#include <iostream>

//#################### HELPER FUNCTIONS ####################
void check_close(const float *v1, const float *v2, size_t size, float tolerance)
{
  for(size_t i = 0; i < size; ++i)
  {
    BOOST_CHECK_CLOSE(v1[i], v2[i], tolerance);
  }
}

//#################### TESTS ####################

BOOST_AUTO_TEST_SUITE(test_AttitudeUtil)

BOOST_AUTO_TEST_CASE(test_conversions)
{
  const float TOL = 1e-4;

  // Start from an axis-angle representation.
  float angle = M_PI/2;
  Vector3f axis(10.0f, 1.0f, 1.0f);

  // The axis vector must be of unit length.
  axis = axis / sqrt(dot(axis,axis));
  //std::cout << "Axis: " << axis << " Angle: " << angle << '\n';

  // Start conversions.
  Vector3f rotationVector;
  AttitudeUtil::axis_angle_to_rotation_vector(axis.v, &angle, rotationVector.v);
  //std::cout << "Rotation vector: " << rotationVector << '\n';

  Vector4f quaternion;
  AttitudeUtil::rotation_vector_to_quaternion(rotationVector.v, quaternion.v);
  //std::cout << "Quaternion: " << quaternion << '\n';

  Vector3f rotationVector2;
  AttitudeUtil::quaternion_to_rotation_vector(quaternion.v, rotationVector2.v);
  //std::cout << "Rotation vector: " << rotationVector2 << '\n';

  Matrix3f rotationMatrix;
  AttitudeUtil::quaternion_to_rotation_matrix(quaternion.v, rotationMatrix.m);
  //std::cout << "Rotation matrix: \n" << rotationMatrix << '\n';

  Matrix3f rotationMatrix2;
  AttitudeUtil::axis_angle_to_rotation_matrix(axis.v, &angle, rotationMatrix2.m);
  //std::cout << "Rotation matrix: \n" << rotationMatrix2 << '\n';

  Vector4f quaternion2;
  AttitudeUtil::rotation_matrix_to_quaternion(rotationMatrix.m, quaternion2.v);
  //std::cout << "Quaternion: " << quaternion2 << '\n';

  Vector3f axis2;
  float angle2;
  AttitudeUtil::quaternion_to_axis_angle(quaternion2.v, axis2.v, &angle2);
  //std::cout << "Axis: " << axis2 << " Angle: " << angle2 << '\n';

  Vector3f axis3;
  float angle3;
  AttitudeUtil::rotation_matrix_to_axis_angle(rotationMatrix.m, axis3.v, &angle3);
  //std::cout << "Axis: " << axis3 << " Angle: " << angle3 << '\n';

  Vector3f axis4;
  float angle4;
  AttitudeUtil::rotation_vector_to_axis_angle(rotationVector.v, axis4.v, &angle4);
  //std::cout << "Axis: " << axis4 << " Angle: " << angle4 << '\n';

  // Check conversions.
  check_close(rotationVector.v, rotationVector2.v, rotationVector.size(), TOL);
  check_close(rotationMatrix.m, rotationMatrix.m, 9, TOL);
  check_close(quaternion.v, quaternion2.v, quaternion.size(), TOL);
  check_close(&angle, &angle2, 1, TOL);
  check_close(&angle, &angle3, 1, TOL);
  check_close(&angle, &angle4, 1, TOL);
  check_close(axis, axis2, axis.size(), TOL);
  check_close(axis, axis3, axis.size(), TOL);
  check_close(axis, axis4, axis.size(), TOL);
}

BOOST_AUTO_TEST_SUITE_END()
