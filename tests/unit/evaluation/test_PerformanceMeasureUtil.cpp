#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <cmath>
#include <vector>

#include <boost/lexical_cast.hpp>
#include <boost/assign/list_of.hpp>
using boost::assign::list_of;
using boost::assign::map_list_of;

#include <evaluation/core/PerformanceMeasureUtil.h>
using namespace evaluation;

#include <tvgutil/containers/MapUtil.h>
using namespace tvgutil;

//#################### TESTS ####################

BOOST_AUTO_TEST_SUITE(test_PerformanceMeasureUtil)

BOOST_AUTO_TEST_CASE(average_measures_test)
{
  const float TOL = 1e-5f;

  #define CHECK_CLOSE(L,R) BOOST_CHECK_CLOSE(L,R,TOL)

  PerformanceMeasure m1 = PerformanceMeasureUtil::average_measures(list_of(1.0f)(2.0f)(3.0f));
  CHECK_CLOSE(m1.get_mean(), 2.0f);
  BOOST_CHECK_EQUAL(m1.get_sample_count(), 3);
  CHECK_CLOSE(m1.get_std_dev(), sqrtf(2.0f/3.0f));
  CHECK_CLOSE(m1.get_variance(), 2.0f/3.0f);

  PerformanceMeasure m2 = PerformanceMeasureUtil::average_measures(list_of(4.0f)(5.0f));
  CHECK_CLOSE(m2.get_mean(), 4.5f);
  BOOST_CHECK_EQUAL(m2.get_sample_count(), 2);
  CHECK_CLOSE(m2.get_std_dev(), 0.5f);
  CHECK_CLOSE(m2.get_variance(), 0.25f);

  PerformanceMeasure avg = PerformanceMeasureUtil::average_measures(list_of(m1)(m2));
  CHECK_CLOSE(avg.get_mean(), 3.0f);
  BOOST_CHECK_EQUAL(avg.get_sample_count(), 5);
  CHECK_CLOSE(avg.get_std_dev(), sqrtf(2.0f));
  CHECK_CLOSE(avg.get_variance(), 2.0f);

  #undef CHECK_CLOSE
}

BOOST_AUTO_TEST_CASE(average_results_test)
{
  PerformanceResult r1 = map_list_of("A",1.0f)("B",2.0f);
  PerformanceResult r2 = map_list_of("A",2.0f)("B",1.0f);
  PerformanceResult r3 = map_list_of("C",3.0f);

  std::vector<PerformanceResult> results = list_of(r1)(r2)(r3);
  PerformanceResult rAvg = PerformanceMeasureUtil::average_results(results);

  BOOST_CHECK_EQUAL(boost::lexical_cast<std::string>(MapUtil::lookup(rAvg, "A")),"1.5 +/- 0.5 (2 samples)");
  BOOST_CHECK_EQUAL(boost::lexical_cast<std::string>(MapUtil::lookup(rAvg, "B")),"1.5 +/- 0.5 (2 samples)");
  BOOST_CHECK_EQUAL(boost::lexical_cast<std::string>(MapUtil::lookup(rAvg, "C")),"3 +/- 0 (1 samples)");
}

BOOST_AUTO_TEST_SUITE_END()
