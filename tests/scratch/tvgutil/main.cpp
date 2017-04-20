//###
#if 1

#include <iostream>
#include <map>
#include <string>

#include <boost/any.hpp>
#include <boost/lexical_cast.hpp>

#include <tvgutil/containers/MapUtil.h>
using namespace tvgutil;

class AppParams
{
private:
  typedef std::map<std::string,std::string> Base;
  Base m_base;
public:
  typedef std::string KeyType;
  typedef std::string ValueType;

  template <typename T>
  void insert(const KeyType& key, const T& value)
  {
    m_base.insert(std::make_pair(key, boost::lexical_cast<std::string>(value)));
  }

  operator const Base&() const
  {
    return m_base;
  }
};

int main()
{
  AppParams ps;
  ps.insert("Foo", 23);
  ps.insert("Bar", "Wibble");
  int i = MapUtil::typed_lookup(ps, "Foo", i);
  std::string s = MapUtil::typed_lookup(ps, "Bar", s);
  std::cout << i << ' ' << s << '\n';
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
