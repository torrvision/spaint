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

std::vector<float> make_test_vector()
{
  std::vector<float> v(5);
  v[0] = 3.2f;
  v[1] = 43.9f;
  v[2] = 0.982f;
  v[3] = 5.67f;
  v[4] = 25.0f;
  return v;
}

//#################### TESTS ####################

BOOST_AUTO_TEST_SUITE(test_ArgUtil)

BOOST_AUTO_TEST_CASE(argmax_map_test)
{
  std::map<std::string,int> m = make_test_map();
  BOOST_CHECK_EQUAL(ArgUtil::argmax(m), "Wibble");
}

BOOST_AUTO_TEST_CASE(argmin_map_test)
{
  std::map<std::string,int> m = make_test_map();
  BOOST_CHECK_EQUAL(ArgUtil::argmin(m), "Bar");
}

BOOST_AUTO_TEST_CASE(argmax_vector_test)
{
  std::vector<float> v = make_test_vector();
  BOOST_CHECK_EQUAL(ArgUtil::argmax(v), 1);
}

BOOST_AUTO_TEST_CASE(argmin_vector_test)
{
  std::vector<float> v = make_test_vector();
  BOOST_CHECK_EQUAL(ArgUtil::argmin(v), 2);
}

BOOST_AUTO_TEST_SUITE_END()
