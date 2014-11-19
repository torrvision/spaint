/**
 * test-tvgutil: main.cpp
 */

#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

// If we're in Visual Studio, redirect std::cout to the output window (this allows us to jump to failing tests more easily).
// Note: This was originally obtained from http://stackoverflow.com/questions/73286/capturing-cout-in-visual-studio-2005-output-window.
#ifdef _MSC_VER

#include <iostream>
#include <vector>

#include <windows.h>

#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/tee.hpp>

class DebugSink
{
  //#################### TYPEDEFS ####################
public:
  typedef boost::iostreams::sink_tag category;
  typedef char char_type;

  //#################### PRIVATE VARIABLES ####################
private:
  std::vector<char> m_vec;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  std::streamsize write(const char *s, std::streamsize n)
  {
    m_vec.assign(s, s+n);
    m_vec.push_back(0); // we must null-terminate for WINAPI
    OutputDebugStringA(&m_vec[0]);
    return n;
  }
};

struct Setup
{
  //#################### CONSTRUCTORS ####################
  Setup()
  {
    typedef boost::iostreams::tee_device<DebugSink, std::streambuf> TeeDevice;
    static TeeDevice device(DebugSink(), *std::cout.rdbuf());
    static boost::iostreams::stream_buffer<TeeDevice> buf(device);
    std::cout.rdbuf(&buf);
  }
};

BOOST_GLOBAL_FIXTURE(Setup)

#endif
