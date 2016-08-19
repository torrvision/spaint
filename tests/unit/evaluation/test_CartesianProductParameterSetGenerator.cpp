#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <boost/assign/list_of.hpp>
using boost::assign::list_of;

#include <evaluation/util/CartesianProductParameterSetGenerator.h>
using namespace evaluation;

//#################### TESTS ####################

BOOST_AUTO_TEST_SUITE(test_CartesianProductParameterSetGenerator)

BOOST_AUTO_TEST_CASE(param_set_to_string_test)
{
  std::vector<ParamSet> settings = CartesianProductParameterSetGenerator()
    .add_param("BumbleBees", list_of<int>(3)(5)(11))
    .add_param("Algorithm", list_of<std::string>("Foo")("Bar"))
    .add_param("Dummies", list_of<float>(9.93f))
    .generate_param_sets();

  std::vector<std::string> actualStrings;
  for(size_t i = 0, size = settings.size(); i < size; ++i)
  {
    actualStrings.push_back(ParamSetUtil::param_set_to_string(settings[i]));
  }

  std::vector<std::string> expectedStrings = list_of
    ("Algorithm-Foo_BumbleBees-3_Dummies-9.93")
    ("Algorithm-Bar_BumbleBees-3_Dummies-9.93")
    ("Algorithm-Foo_BumbleBees-5_Dummies-9.93")
    ("Algorithm-Bar_BumbleBees-5_Dummies-9.93")
    ("Algorithm-Foo_BumbleBees-11_Dummies-9.93")
    ("Algorithm-Bar_BumbleBees-11_Dummies-9.93");

  for(size_t i = 0, size = settings.size(); i < size; ++i)
  {
    BOOST_CHECK_EQUAL(actualStrings[i], expectedStrings[i]);
  }
}

BOOST_AUTO_TEST_SUITE_END()
