#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

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

BOOST_AUTO_TEST_CASE(average_test)
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
