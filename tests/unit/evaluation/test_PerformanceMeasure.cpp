#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <cmath>
#include <vector>

#include <boost/assign/list_of.hpp>
using boost::assign::list_of;

#include <evaluation/core/PerformanceMeasure.h>
using namespace evaluation;

//#################### TESTS ####################

BOOST_AUTO_TEST_SUITE(test_PerformanceMeasure)

BOOST_AUTO_TEST_CASE(average_test)
{
  const float TOL = 1e-5f;
  #define CHECK_CLOSE(L,R) BOOST_CHECK_CLOSE(L,R,TOL)

  PerformanceMeasure m1 = PerformanceMeasure::average(list_of(1.0f)(2.0f)(3.0f));
  CHECK_CLOSE(m1.get_mean(), 2.0f);
  BOOST_CHECK_EQUAL(m1.get_sample_count(), 3);
  CHECK_CLOSE(m1.get_std_dev(), sqrtf(2.0f/3.0f));
  CHECK_CLOSE(m1.get_variance(), 2.0f/3.0f);

  PerformanceMeasure m2 = PerformanceMeasure::average(list_of(4.0f)(5.0f));
  CHECK_CLOSE(m2.get_mean(), 4.5f);
  BOOST_CHECK_EQUAL(m2.get_sample_count(), 2);
  CHECK_CLOSE(m2.get_std_dev(), 0.5f);
  CHECK_CLOSE(m2.get_variance(), 0.25f);

  PerformanceMeasure avg = PerformanceMeasure::average(list_of(m1)(m2));
  CHECK_CLOSE(avg.get_mean(), 3.0f);
  BOOST_CHECK_EQUAL(avg.get_sample_count(), 5);
  CHECK_CLOSE(avg.get_std_dev(), sqrtf(2.0f));
  CHECK_CLOSE(avg.get_variance(), 2.0f);

  #undef CHECK_CLOSE
}

BOOST_AUTO_TEST_SUITE_END()
