#include <iostream>

#include <boost/asio.hpp>
using namespace boost::asio;

int main()
{
  ip::tcp::iostream s("localhost", "23984");

  std::string line;
  while(s)
  {
    size_t availableBytes;
    while((availableBytes = s.rdbuf()->available()) > 0)
    {
      std::cout << availableBytes << '\n';
      std::getline(s, line);
      std::cout << line << '\n';
    }
  }

  return 0;
}
