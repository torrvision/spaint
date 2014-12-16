#if 0
int main()
{
  // TODO
  return 0;
}
#endif

#if 1

#include <iostream>
#include <string>

#include <tvgutil/Timing.h>

class Dummy
{
public:
  Dummy(size_t time)
  {
    std::cout << "I'm very tired.. going for a nap of " << std::to_string(time) << " seconds..\n";
    sleep(time);
  }
  void snooze(size_t time)
  {
    std::cout << "I'm very tired.. going for a snooze of " << std::to_string(time) << " seconds..\n";
    sleep(time);
  }
};


int main()
{
  std::cout << "Testing the timing features!\n";

  TIMEX(Dummy d1(2), milliseconds, dummyConstructor)
  dummyConstructor.print();

  TIMEX(d1.snooze(3), milliseconds, dummySnooze)
  dummySnooze.print();

  return 0;
}
#endif
