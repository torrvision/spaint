#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <map>
#include <set>
#include <sstream>
#include <vector>

#include <tvgutil/LimitedContainer.h>
using namespace tvgutil;

BOOST_AUTO_TEST_SUITE(test_LimitedContainer)

BOOST_AUTO_TEST_CASE(map_test)
{
  std::map<int,int> m;
  for(int i = 0; i < 5; ++i)
  {
    m[i] = i * i;
  }

  BOOST_CHECK_THROW(make_limited_container(m, 1), std::runtime_error);

  {
    std::stringstream ss;
    ss << make_limited_container(m, 2);
    BOOST_CHECK_EQUAL(ss.str(), "[ (0,0) ... (4,16) ]");
  }

  {
    std::stringstream ss;
    ss << make_limited_container(m, 3);
    BOOST_CHECK_EQUAL(ss.str(), "[ (0,0) (1,1) ... (4,16) ]");
  }

  {
    std::stringstream ss;
    ss << make_limited_container(m, 4);
    BOOST_CHECK_EQUAL(ss.str(), "[ (0,0) (1,1) (2,4) ... (4,16) ]");
  }

  {
    std::stringstream ss;
    ss << make_limited_container(m, 5);
    BOOST_CHECK_EQUAL(ss.str(), "[ (0,0) (1,1) (2,4) (3,9) (4,16) ]");
  }
}

BOOST_AUTO_TEST_CASE(set_test)
{
  std::set<float> s;
  for(int i = 0; i < 5; ++i)
  {
    s.insert(4.0f - i);
  }

  BOOST_CHECK_THROW(make_limited_container(s, 1), std::runtime_error);

  {
    std::stringstream ss;
    ss << make_limited_container(s, 2);
    BOOST_CHECK_EQUAL(ss.str(), "[ 0 ... 4 ]");
  }

  {
    std::stringstream ss;
    ss << make_limited_container(s, 3);
    BOOST_CHECK_EQUAL(ss.str(), "[ 0 1 ... 4 ]");
  }

  {
    std::stringstream ss;
    ss << make_limited_container(s, 4);
    BOOST_CHECK_EQUAL(ss.str(), "[ 0 1 2 ... 4 ]");
  }

  {
    std::stringstream ss;
    ss << make_limited_container(s, 5);
    BOOST_CHECK_EQUAL(ss.str(), "[ 0 1 2 3 4 ]");
  }
}

BOOST_AUTO_TEST_CASE(vector_test)
{
  std::vector<std::string> v;
  v.push_back("Foo");
  v.push_back("Bar");
  v.push_back("Baz");
  v.push_back("Wibble");
  v.push_back("Wobble");

  BOOST_CHECK_THROW(make_limited_container(v, 1), std::runtime_error);

  {
    std::stringstream ss;
    ss << make_limited_container(v, 2);
    BOOST_CHECK_EQUAL(ss.str(), "[ Foo ... Wobble ]");
  }

  {
    std::stringstream ss;
    ss << make_limited_container(v, 3);
    BOOST_CHECK_EQUAL(ss.str(), "[ Foo Bar ... Wobble ]");
  }

  {
    std::stringstream ss;
    ss << make_limited_container(v, 4);
    BOOST_CHECK_EQUAL(ss.str(), "[ Foo Bar Baz ... Wobble ]");
  }

  {
    std::stringstream ss;
    ss << make_limited_container(v, 5);
    BOOST_CHECK_EQUAL(ss.str(), "[ Foo Bar Baz Wibble Wobble ]");
  }
}

BOOST_AUTO_TEST_SUITE_END()
