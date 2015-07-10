/**
 * tvgutil: ExecutableFinder.cpp
 */

#include "ExecutableFinder.h"

#include <vector>

#if defined(_WIN32)
  #include <Windows.h>
#elif defined(__linux__)
  // TODO
#endif

namespace tvgutil {

boost::filesystem::path find_executable()
{
  const int BUFFER_SIZE = 512;
  std::vector<char> buffer(BUFFER_SIZE + 1);

#if defined(_WIN32)
  ::GetModuleFileName(NULL, &buffer[0], BUFFER_SIZE);
#elif defined(__linux__)
  #error Cannot yet find the executable on this platform
#elif defined(__APPLE__)
  #error Cannot yet find the executable on this platform
#else
  #error Cannot yet find the executable on this platform
#endif

  std::string s = &buffer[0];
  return s;
}

}
