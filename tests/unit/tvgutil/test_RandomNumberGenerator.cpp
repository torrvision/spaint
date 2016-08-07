#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <tvgutil/numbers/RandomNumberGenerator.h>
using namespace tvgutil;

BOOST_AUTO_TEST_SUITE(test_RandomNumberGenerator)

BOOST_AUTO_TEST_CASE(generate_int_from_uniform_test)
{
  RandomNumberGenerator rng(1234);
  BOOST_CHECK_EQUAL(rng.generate_int_from_uniform(23,23), 23);
}

BOOST_AUTO_TEST_SUITE_END()
