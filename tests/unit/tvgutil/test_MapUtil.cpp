#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <sstream>

#include <boost/bind.hpp>

#include <tvgutil/containers/MapUtil.h>
using namespace tvgutil;

//#################### HELPER TYPES ####################

struct S
{
  int i;

  S() {}
  S(int i_) : i(i_) {}

  void f(std::ostream& os)
  {
    os << "f: " << i;
  }

  void g(std::ostream& os) const
  {
    os << "g: " << i;
  }
};

//#################### TESTS ####################

BOOST_AUTO_TEST_SUITE(test_MapUtil)

BOOST_AUTO_TEST_CASE(call_if_found_test)
{
  std::map<std::string,S> m;
  const std::map<std::string,S>& cm = m;

  m["Foo"] = 23;
  m["Bar"] = 9;

  {
    std::ostringstream oss;
    MapUtil::call_if_found(m, "Foo", boost::bind(&S::f, _1, boost::ref(oss)));
    BOOST_CHECK_EQUAL(oss.str(), "f: 23");
  }

  {
    std::ostringstream oss;
    MapUtil::call_if_found(m, "Bar", boost::bind(&S::g, _1, boost::ref(oss)));
    BOOST_CHECK_EQUAL(oss.str(), "g: 9");
  }

  {
    std::ostringstream oss;
    MapUtil::call_if_found(cm, "Foo", boost::bind(&S::g, _1, boost::ref(oss)));
    BOOST_CHECK_EQUAL(oss.str(), "g: 23");
  }
}

BOOST_AUTO_TEST_SUITE_END()
