#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <ITMLib/Utils/ITMMath.h>

#include <tvgutil/AttitudeUtil.h>
using namespace tvgutil;

//#################### HELPER FUNCTIONS ####################

void check_close(float a, float b, float TOL)
{
  BOOST_CHECK_CLOSE(a, b, TOL);
}

void check_close(const float *v1, const float *v2, size_t size, float TOL)
{
  for(size_t i = 0; i < size; ++i) check_close(v1[i], v2[i], TOL);
}

void check_close(Vector3f v1, Vector3f v2, float TOL)
{
  check_close(v1.v, v2.v, v1.size(), TOL);
}

void check_close(Vector4f v1, Vector4f v2, float TOL)
{
  check_close(v1.v, v2.v, v1.size(), TOL);
}

void check_close(const Matrix3f& m1, const Matrix3f& m2, float TOL)
{
  check_close(m1.m, m2.m, 9, TOL);
}

//#################### TESTS ####################

BOOST_AUTO_TEST_SUITE(test_AttitudeUtil)

BOOST_AUTO_TEST_CASE(test_corner_cases)
{
  const float TOL = 1e-10;

  float angle = 0.0f;
  Vector3f axis(1.0f, 0.0f, 0.0f);

  Vector3f rotationVector;
  AttitudeUtil::axis_angle_to_rotation_vector(axis.v, &angle, rotationVector.v);
  check_close(rotationVector, Vector3f(0.0f, 0.0f, 0.0f), TOL);
  
  float angle2;
  Vector3f axis2;
  AttitudeUtil::rotation_vector_to_axis_angle(rotationVector.v, axis2.v, &angle2);

  // By convension the zero rotation vector corresponds to a zero rotation around the x-axis.
  check_close(angle2, 0.0f, TOL);
  check_close(axis2, Vector3f(1.0f, 0.0f, 0.0f), TOL);

  Vector4f q;
  AttitudeUtil::rotation_vector_to_quaternion(rotationVector.v, q.v);
  check_close(q, Vector4f(1.0f, 0.0f, 0.0f, 0.0f), TOL);

  float angle3;
  Vector3f axis3;
  AttitudeUtil::quaternion_to_axis_angle(q.v, axis3.v, &angle3);
  check_close(angle3, 0.0f, TOL);
  check_close(axis3, Vector3f(1.0f, 0.0f, 0.0f), TOL);
}


BOOST_AUTO_TEST_CASE(test_conversions)
{
  const float TOL = 1e-4;

  // Start from an axis-angle representation.
  float angle = M_PI/4.0f;
  Vector3f axis(1.0f, 0.0f, 0.0f);

  // The axis vector must be of unit length.
  axis = axis / sqrt(dot(axis,axis));

  // Start conversions.
  Vector3f rotationVector;
  AttitudeUtil::axis_angle_to_rotation_vector(axis.v, &angle, rotationVector.v);
  check_close(rotationVector, Vector3f(M_PI/4.0f, 0.0f, 0.0f), TOL);

  Vector4f quaternion;
  AttitudeUtil::rotation_vector_to_quaternion(rotationVector.v, quaternion.v);
  check_close(quaternion.v, Vector4f(0.9238795f, 0.3826834f, 0.0f, 0.0f), TOL);

  Vector3f rotationVector2;
  AttitudeUtil::quaternion_to_rotation_vector(quaternion.v, rotationVector2.v);
  check_close(rotationVector, rotationVector2, TOL);

  Matrix3f rotationMatrix;
  AttitudeUtil::quaternion_to_rotation_matrix(quaternion.v, rotationMatrix.m, AttitudeUtil::COL_MAJOR);
  const float root2inv = 1.0f / sqrt(2.0f);
  check_close(rotationMatrix, Matrix3f(1.0f, 0.0f, 0.0f,
                                       0.0f, root2inv, -root2inv,
                                       0.0f, root2inv,  root2inv), TOL);

  Matrix3f rotationMatrix2;
  AttitudeUtil::axis_angle_to_rotation_matrix(axis.v, &angle, rotationMatrix2.m, AttitudeUtil::COL_MAJOR);
  check_close(rotationMatrix, rotationMatrix2, TOL);

  Vector4f quaternion2;
  AttitudeUtil::rotation_matrix_to_quaternion(rotationMatrix.m, quaternion2.v, AttitudeUtil::COL_MAJOR);
  check_close(quaternion, quaternion, TOL);

  Vector3f axis2;
  float angle2;
  AttitudeUtil::quaternion_to_axis_angle(quaternion2.v, axis2.v, &angle2);
  check_close(angle, angle2, TOL);
  check_close(axis, axis2, TOL);

  Vector3f axis3;
  float angle3;
  AttitudeUtil::rotation_matrix_to_axis_angle(rotationMatrix.t().getValues(), axis3.v, &angle3, AttitudeUtil::ROW_MAJOR);
  check_close(angle, angle3, TOL);
  check_close(axis, axis3, TOL);

  Vector3f axis4;
  float angle4;
  AttitudeUtil::rotation_vector_to_axis_angle(rotationVector.v, axis4.v, &angle4);
  check_close(angle, angle4, TOL);
  check_close(axis, axis4, TOL);
}

BOOST_AUTO_TEST_SUITE_END()
