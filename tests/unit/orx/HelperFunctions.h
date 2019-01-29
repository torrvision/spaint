#include <boost/test/test_tools.hpp>

#include <ORUtils/Math.h>

//#################### HELPER FUNCTIONS ####################

template <typename T>
void check_close(T a, T b, T TOL)
{
  BOOST_CHECK_CLOSE(a, b, TOL);
}

template <typename T>
void check_close(const T *v1, const T *v2, size_t size, T TOL)
{
  for(size_t i = 0; i < size; ++i) check_close(v1[i], v2[i], TOL);
}

template <typename T>
void check_close(const ORUtils::Vector3<T>& v1, const ORUtils::Vector3<T>& v2, T TOL)
{
  check_close(v1.v, v2.v, v1.size(), TOL);
}

template <typename T>
void check_close(const ORUtils::Vector4<T>& v1, const ORUtils::Vector4<T>& v2, T TOL)
{
  check_close(v1.v, v2.v, v1.size(), TOL);
}

template <typename T>
void check_close(const ORUtils::Matrix3<T>& m1, const ORUtils::Matrix3<T>& m2, T TOL)
{
  check_close(m1.m, m2.m, 9, TOL);
}
