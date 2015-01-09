//###
#if 1

#include <iostream>

#include <boost/thread.hpp>

#include <tvgutil/timers/AverageTimer.h>
#include <tvgutil/timers/Timer.h>

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
