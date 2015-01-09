#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <boost/assign/list_of.hpp>
using boost::assign::list_of;

#include <rafl/evaluation/ParameterStringGenerator.h>
using namespace rafl;

BOOST_AUTO_TEST_SUITE(test_ParameterStringGenerator)

BOOST_AUTO_TEST_CASE(get_foo_test)
{
  std::vector<std::string> paramStrings = ParameterStringGenerator()
    .add_param("-n", list_of(3)(4)(5))
    .add_param("-t", list_of<std::string>("Foo")("Bar"))
    .add_param("-x", list_of(23.0f)(9.0f))
    .generate();
  std::copy(paramStrings.begin(), paramStrings.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
 
 //BOOST_CHECK_EQUAL(testConfMtxNormL1, trueConfMtxNormL1);
}

BOOST_AUTO_TEST_SUITE_END()
