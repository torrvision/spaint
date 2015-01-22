#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <boost/assign/list_of.hpp>
using boost::assign::list_of;

#include <rafl/evaluation/ParameterSetProductGenerator.h>
using namespace rafl;

typedef ParameterSetProductGenerator::ParamSet ParamSet;

BOOST_AUTO_TEST_SUITE(test_ParameterSetProductGenerator)

BOOST_AUTO_TEST_CASE(get_foo_test)
{
  std::vector<ParamSet> settings = ParameterSetProductGenerator()
    .add_param("BumbleBees", list_of(3)(5))
    .add_param("Algorithm", list_of<std::string>("Foo")("Bar"))
    .add_param("Dummies", list_of(9.0f)(100.0f))
    .generate_maps();

  std::vector<std::string> settingStrings;
  for(size_t i = 0, iend = settings.size(); i < iend; ++i)
  {
    settingStrings.push_back(ParameterSetProductGenerator::to_string(settings[i]));
  }

  for(size_t i = 0, iend = settingStrings.size(); i < iend; ++i)
  {
    std::cout << settingStrings[i] << "\n";
  }

  std::vector<std::string> trueStrings;
  trueStrings.push_back("-n 3");

 //BOOST_CHECK_EQUAL(testConfMtxNormL1, trueConfMtxNormL1);
}

BOOST_AUTO_TEST_SUITE_END()
