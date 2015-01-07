#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <tvgutil/ArgUtil.h>
using namespace tvgutil;

//#################### HELPER FUNCTIONS ####################

std::map<std::string,int> make_test_map()
{
  std::map<std::string,int> m;
  m["Foo"] = 23;
  m["Bar"] = 9;
  m["Wibble"] = 84;
  m["Wobble"] = 17;
  return m;
}

//#################### TESTS ####################

BOOST_AUTO_TEST_SUITE(test_ArgUtil)

BOOST_AUTO_TEST_CASE(argmax_test)
{
  std::map<std::string,int> m = make_test_map();
  BOOST_CHECK_EQUAL(ArgUtil::argmax(m), "Wibble");
}

BOOST_AUTO_TEST_CASE(argmin_test)
{
  std::map<std::string,int> m = make_test_map();
  BOOST_CHECK_EQUAL(ArgUtil::argmin(m), "Bar");
}

BOOST_AUTO_TEST_SUITE_END()
