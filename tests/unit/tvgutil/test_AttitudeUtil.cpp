#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <ITMLib/Utils/ITMMath.h>

#include <tvgutil/misc/AttitudeUtil.h>
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

void check_close(const Vector3f& v1, const Vector3f& v2, float TOL)
{
  check_close(v1.v, v2.v, v1.size(), TOL);
}

void check_close(const Vector4f& v1, const Vector4f& v2, float TOL)
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
  const float TOL = 1e-4f;

  float angle = 0.0f;
  Vector3f axis(1.0f, 0.0f, 0.0f);

  Vector3f rotationVector;
  AttitudeUtil::axis_angle_to_rotation_vector(axis.v, &angle, rotationVector.v);
  check_close(rotationVector, Vector3f(0.0f, 0.0f, 0.0f), TOL);
  
  {
    float angle;
    Vector3f axis;
    AttitudeUtil::rotation_vector_to_axis_angle(rotationVector.v, axis.v, &angle);

    // By convention the zero rotation vector corresponds to a zero rotation around the x-axis.
    check_close(angle, 0.0f, TOL);
    check_close(axis, Vector3f(1.0f, 0.0f, 0.0f), TOL);
  }

  Vector4f q;
  AttitudeUtil::rotation_vector_to_quaternion(rotationVector.v, q.v);
  check_close(q, Vector4f(1.0f, 0.0f, 0.0f, 0.0f), TOL);

  {
    float angle;
    Vector3f axis;
    AttitudeUtil::quaternion_to_axis_angle(q.v, axis.v, &angle);
    check_close(angle, 0.0f, TOL);
    check_close(axis, Vector3f(1.0f, 0.0f, 0.0f), TOL);
  }
}


BOOST_AUTO_TEST_CASE(test_conversions)
{
  const float TOL = 1e-4f;

  // Start from an axis-angle representation.
  float angle = static_cast<float>(M_PI) / 4.0f;
  Vector3f axis(1.0f, 0.0f, 0.0f);

  // Start conversions.
  Vector3f rotationVector;
  AttitudeUtil::axis_angle_to_rotation_vector(axis.v, &angle, rotationVector.v);
  check_close(rotationVector, Vector3f(static_cast<float>(M_PI) / 4.0f, 0.0f, 0.0f), TOL);

  Vector4f quaternion;
  AttitudeUtil::rotation_vector_to_quaternion(rotationVector.v, quaternion.v);
  check_close(quaternion, Vector4f(0.9238795f, 0.3826834f, 0.0f, 0.0f), TOL);

  {
    Vector3f rotationVector_;
    AttitudeUtil::quaternion_to_rotation_vector(quaternion.v, rotationVector_.v);
    check_close(rotationVector, rotationVector_, TOL);
  }

  Matrix3f rotationMatrix;
  AttitudeUtil::quaternion_to_rotation_matrix(quaternion.v, rotationMatrix.m, AttitudeUtil::COL_MAJOR);
  const float root2inv = 1.0f / sqrt(2.0f);
  check_close(
    rotationMatrix,
    Matrix3f(
      1.0f, 0.0f, 0.0f,
      0.0f, root2inv, -root2inv,
      0.0f, root2inv,  root2inv
    ),
    TOL
  );

  {
    Matrix3f rotationMatrix_;
    AttitudeUtil::axis_angle_to_rotation_matrix(axis.v, &angle, rotationMatrix_.m, AttitudeUtil::COL_MAJOR);
    check_close(rotationMatrix, rotationMatrix_, TOL);
  }

  {
    Vector4f quaternion_;
    AttitudeUtil::rotation_matrix_to_quaternion(rotationMatrix.m, quaternion_.v, AttitudeUtil::COL_MAJOR);
    check_close(quaternion, quaternion_, TOL);
  }

  {
    Vector3f axis_;
    float angle_;
    AttitudeUtil::quaternion_to_axis_angle(quaternion.v, axis_.v, &angle_);
    check_close(angle, angle_, TOL);
    check_close(axis, axis_, TOL);
  }

  {
    Vector3f axis_;
    float angle_;
    AttitudeUtil::rotation_matrix_to_axis_angle(rotationMatrix.t().getValues(), axis_.v, &angle_, AttitudeUtil::ROW_MAJOR);
    check_close(angle, angle_, TOL);
    check_close(axis, axis_, TOL);
  }

  {
    Vector3f axis_;
    float angle_;
    AttitudeUtil::rotation_vector_to_axis_angle(rotationVector.v, axis_.v, &angle_);
    check_close(angle, angle_, TOL);
    check_close(axis, axis_, TOL);
  }
}

BOOST_AUTO_TEST_SUITE_END()
