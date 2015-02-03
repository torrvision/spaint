#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <boost/assign/list_of.hpp>
using boost::assign::list_of;

#include <evaluation/util/CartesianProductParameterSetGenerator.h>
using namespace evaluation;

typedef CartesianProductParameterSetGenerator::ParamSet ParamSet;

BOOST_AUTO_TEST_SUITE(test_CartesianProductParameterSetGenerator)

BOOST_AUTO_TEST_CASE(get_foo_test)
{
  std::vector<ParamSet> settings = CartesianProductParameterSetGenerator()
    .add_param("BumbleBees", list_of<int>(3)(5)(11))
    .add_param("Algorithm", list_of<std::string>("Foo")("Bar"))
    .add_param("Dummies", list_of<float>(9.93f))
    .generate_maps();

  std::vector<std::string> settingStrings;
  for(size_t i = 0, iend = settings.size(); i < iend; ++i)
  {
    settingStrings.push_back(CartesianProductParameterSetGenerator::to_string(settings[i]));
  }

  std::vector<std::string> trueStrings;
  trueStrings.push_back("Algorithm-Foo_BumbleBees-3_Dummies-9.93");
  trueStrings.push_back("Algorithm-Bar_BumbleBees-3_Dummies-9.93");
  trueStrings.push_back("Algorithm-Foo_BumbleBees-5_Dummies-9.93");
  trueStrings.push_back("Algorithm-Bar_BumbleBees-5_Dummies-9.93");
  trueStrings.push_back("Algorithm-Foo_BumbleBees-11_Dummies-9.93");
  trueStrings.push_back("Algorithm-Bar_BumbleBees-11_Dummies-9.93");

  for(size_t i = 0, iend = settingStrings.size(); i < iend; ++i)
  {
    BOOST_CHECK_EQUAL(settingStrings[i], trueStrings[i]);
  }

}

BOOST_AUTO_TEST_SUITE_END()
