#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <ITMLib/Utils/ITMMath.h>

#include <ORUtils/SE3Pose.h>

#include <spaint/geometry/DualQuaternion.h>
using namespace ORUtils;
using namespace spaint;

//#################### HELPER FUNCTIONS ####################

template <typename T>
void from_point_test()
{
  DualQuaternion<T> dq = DualQuaternion<T>::from_point(Vector3<T>(3,4,5));
  BOOST_CHECK_EQUAL(dq.conjugate(), DualQuaternion<T>(DualNumber<T>(1,0), DualNumber<T>(0,-3), DualNumber<T>(0,-4), DualNumber<T>(0,-5)));
  BOOST_CHECK_EQUAL(dq.norm(), DualNumber<T>(1,0));
}

template <typename T>
void from_rotation_test()
{
  Vector3<T> axis(0,1,0);
  DualQuaternion<T> dq = DualQuaternion<T>::from_rotation(axis, T(M_PI_4));

  BOOST_CHECK_SMALL(length(dq.get_rotation() - axis * T(M_PI_4)), T(1e-4));
  BOOST_CHECK_EQUAL(dq.get_rotation_part(), dq);
  BOOST_CHECK_SMALL(length(dq.get_translation()), T(1e-4));

  Vector3<T> v(1,0,0);
  Vector3<T> w1 = dq.apply(v);

  SE3Pose pose(0, 0, 0, 0, static_cast<float>(M_PI_4), 0);
  Vector3f t, r;
  pose.GetParams(t, r);
  Vector3<T> w2 = DualQuaternion<T>::from_rotation(r).apply(v);

  BOOST_CHECK_SMALL(length(w2 - w1), T(1e-4));
}

template <typename T>
void from_se3_test()
{
  DualQuaternion<T> rot = DualQuaternion<T>::from_rotation(Vector3<T>(0,0,1), T(M_PI_2));
  DualQuaternion<T> trans = DualQuaternion<T>::from_translation(Vector3<T>(3,4,5));
  Vector3<T> v(1,0,0);

  BOOST_CHECK_EQUAL(rot, DualQuaternion<T>(DualNumber<T>(0.707107f,0), DualNumber<T>(0,0), DualNumber<T>(0,0), DualNumber<T>(0.707107f,0)));
  BOOST_CHECK_EQUAL(trans, DualQuaternion<T>(DualNumber<T>(1,0), DualNumber<T>(0,1.5f), DualNumber<T>(0,2), DualNumber<T>(0,2.5f)));
  BOOST_CHECK_EQUAL(trans * rot, DualQuaternion<T>(DualNumber<T>(0.707107f,-1.76777f), DualNumber<T>(0,2.47487f), DualNumber<T>(0,0.353553f), DualNumber<T>(0.707107f,1.76777f)));
  BOOST_CHECK_SMALL(length(rot.apply(v) - Vector3<T>(0,1,0)), T(1e-4));
  BOOST_CHECK_SMALL(length(trans.apply(v) - Vector3<T>(4,4,5)), T(1e-4));
  BOOST_CHECK_SMALL(length((trans * rot).apply(v) - Vector3<T>(3,5,5)), T(1e-4));
  BOOST_CHECK_SMALL(length((rot * trans).apply(v) - Vector3<T>(-4,4,5)), T(1e-4));
}

template <typename T>
void from_translation_test()
{
  Vector3<T> v(1,2,3), t(3,4,5);
  DualQuaternion<T> dq = DualQuaternion<T>::from_translation(t);
  BOOST_CHECK_EQUAL(dq.apply(v), v + t);
  BOOST_CHECK_EQUAL(dq.get_rotation(), Vector3<T>(0,0,0));
  BOOST_CHECK_EQUAL(dq.get_rotation_part(), DualQuaternion<T>::from_rotation(Vector3<T>(1,0,0), T(0)));
  BOOST_CHECK_EQUAL(dq.get_translation(), t);
  BOOST_CHECK_EQUAL(dq.get_translation_part(), dq);
}

template <typename T>
void linear_blend_test()
{
  DualQuaternion<T> p = DualQuaternion<T>::from_translation(Vector3<T>(2,3,4));
  DualQuaternion<T> q = DualQuaternion<T>::from_translation(Vector3<T>(3,4,4)) * DualQuaternion<T>::from_rotation(Vector3<T>(0,0,1), T(M_PI_2));
  Vector3<T> v(1,0,0);

  std::vector<DualQuaternion<T> > dqs;
  dqs.push_back(p);
  dqs.push_back(q);

  std::vector<T> weights(2);

  weights[0] = 1.0f, weights[1] = 0.0f;
  BOOST_CHECK_SMALL(length(DualQuaternion<T>::linear_blend(&dqs[0], &weights[0], static_cast<int>(dqs.size())).apply(v) - Vector3<T>(3,3,4)), T(1e-4));

  weights[0] = 0.5f, weights[1] = 0.5f;
  BOOST_CHECK_SMALL(length(DualQuaternion<T>::linear_blend(&dqs[0], &weights[0], static_cast<int>(dqs.size())).apply(v) - Vector3<T>(3.41421f,4,4)), T(1e-4));

  weights[0] = 0.0f, weights[1] = 1.0f;
  BOOST_CHECK_SMALL(length(DualQuaternion<T>::linear_blend(&dqs[0], &weights[0], static_cast<int>(dqs.size())).apply(v) - Vector3<T>(3,5,4)), T(1e-4));
}

template <typename T>
void pow_test()
{
  DualQuaternion<T> rot = DualQuaternion<T>::from_rotation(Vector3<T>(0,0,1), T(M_PI_2));
  DualQuaternion<T> tripleRot = rot.pow(3);
  Vector3<T> v(1,0,0);
  BOOST_CHECK_SMALL(length(tripleRot.apply(v) - Vector3<T>(0,-1,0)), T(1e-4));

  DualQuaternion<T> trans = DualQuaternion<T>::from_translation(Vector3<T>(3,4,5));
  DualQuaternion<T> tr = trans * rot;
  BOOST_CHECK_SMALL(length(tr.pow(3).apply(v) - (tr * tr * tr).apply(v)), T(1e-4));
}

template <typename T>
void sclerp_test()
{
  DualQuaternion<T> p = DualQuaternion<T>::from_translation(Vector3<T>(2,3,4));
  DualQuaternion<T> q = DualQuaternion<T>::from_translation(Vector3<T>(3,4,4)) * DualQuaternion<T>::from_rotation(Vector3<T>(0,0,1), T(M_PI_2));
  BOOST_CHECK_EQUAL(p, DualQuaternion<T>(DualNumber<T>(1,0), DualNumber<T>(0,1), DualNumber<T>(0,1.5f), DualNumber<T>(0,2)));
  BOOST_CHECK_EQUAL(q, DualQuaternion<T>(DualNumber<T>(0.707107f,-1.41421f), DualNumber<T>(0,2.47487f), DualNumber<T>(0,0.353553f), DualNumber<T>(0.707107f,1.41421f)));

  Vector3<T> v(1,0,0);
  BOOST_CHECK_SMALL(length(DualQuaternion<T>::sclerp(p, q, 0.0).apply(v) - Vector3<T>(3,3,4)), T(1e-4));
  BOOST_CHECK_SMALL(length(DualQuaternion<T>::sclerp(p, q, 0.5).apply(v) - Vector3<T>(3.41421f,4,4)), T(1e-4));
  BOOST_CHECK_SMALL(length(DualQuaternion<T>::sclerp(p, q, 1.0).apply(v) - Vector3<T>(3,5,4)), T(1e-4));
}

template <typename T>
void screw_test()
{
  DualQuaternion<T> rot = DualQuaternion<T>::from_rotation(Vector3<T>(0,0,1), T(M_PI_2));
  DualQuaternion<T> trans = DualQuaternion<T>::from_translation(Vector3<T>(3,4,5));
  DualQuaternion<T> tr = trans * rot;
  BOOST_CHECK_EQUAL(DualQuaternion<T>::from_screw(tr.to_screw()), tr);
}

//#################### TESTS ####################

BOOST_AUTO_TEST_SUITE(test_DualQuaternion)

BOOST_AUTO_TEST_CASE(test_from_point)
{
  from_point_test<double>();
  from_point_test<float>();
}

BOOST_AUTO_TEST_CASE(test_from_rotation)
{
  from_rotation_test<double>();
  from_rotation_test<float>();
}

BOOST_AUTO_TEST_CASE(test_from_se3)
{
  from_se3_test<double>();
  from_se3_test<float>();
}

BOOST_AUTO_TEST_CASE(test_from_translation)
{
  from_translation_test<double>();
  from_translation_test<float>();
}

BOOST_AUTO_TEST_CASE(test_linear_blend)
{
  linear_blend_test<double>();
  linear_blend_test<float>();
}

BOOST_AUTO_TEST_CASE(test_pow)
{
  pow_test<double>();
  pow_test<float>();
}

BOOST_AUTO_TEST_CASE(test_sclerp)
{
  sclerp_test<double>();
  sclerp_test<float>();
}

BOOST_AUTO_TEST_CASE(test_screw)
{
  screw_test<double>();
  screw_test<float>();
}

BOOST_AUTO_TEST_SUITE_END()
