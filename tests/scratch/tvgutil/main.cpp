//###
#if 1

#include <algorithm>
#include <iostream>
#include <iterator>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/any.hpp>
#include <boost/lexical_cast.hpp>

#include <tvgutil/containers/MapUtil.h>
using namespace tvgutil;

class AppParams
{
private:
  typedef std::map<std::string,boost::any> Base;
  Base m_base;
public:
  template <typename V>
  void insert(const std::string& key, const V& value)
  {
    m_base.insert(std::make_pair(key, value));
  }

  void insert(const std::string& key, const char *value)
  {
    m_base.insert(std::make_pair(key, std::string(value)));
  }

  template <typename V>
  V operator()(const std::string& key, const V& /* dummy */) const
  {
    Base::const_iterator it = m_base.find(key);
    if(it != m_base.end()) return boost::any_cast<V>(it->second);
    else throw std::runtime_error("...");
  }
};

int main()
{
  AppParams ps;
  ps.insert("Foo", 23);
  ps.insert("Bar", "Wibble");
  std::vector<std::string> vec;
  vec.push_back("9");
  ps.insert("Baz", vec);
  int i = ps("Foo", i);
  std::string s = ps("Bar", s);
  std::vector<std::string> v = ps("Baz", v);
  std::cout << i << ' ' << s << '\n';
  std::copy(v.begin(), v.end(), std::ostream_iterator<std::string>(std::cout, " "));
  return 0;
}

#endif

//###
#if 0

#include <iostream>

#include <boost/thread.hpp>

#include <tvgutil/timing/AverageTimer.h>
#include <tvgutil/timing/Timer.h>

class Dummy
{
public:
  Dummy(long time)
  {
    std::cout << "I'm very tired.. going for a nap of " << time << " seconds..\n";
    boost::this_thread::sleep(boost::posix_time::seconds(time));
  }
  void snooze(long time)
  {
    std::cout << "I'm very tired.. going for a snooze of " << time << " seconds..\n";
    boost::this_thread::sleep(boost::posix_time::seconds(time));
  }
};


int main()
{
  std::cout << "Testing the timing features!\n";

  TIME(Dummy d1(2), milliseconds, dummyConstructor);
  std::cout << dummyConstructor << '\n';

  TIME(d1.snooze(3), milliseconds, dummySnooze);
  std::cout << dummySnooze << '\n';

  for(int i = 1; i <= 5; ++i)
  {
    AVG_TIME(d1.snooze(i), microseconds, avgSnooze);
    std::cout << avgSnooze.last_duration() << ' ' << avgSnooze.total_duration() << ' ' << avgSnooze.count() << ' ' << avgSnooze.average_duration() << '\n';
  }

  return 0;
}

#endif
