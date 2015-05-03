#include <iostream>

#include <boost/asio.hpp>
using namespace boost::asio;

int main()
{
  ip::tcp::iostream s("localhost", "23984");

  std::string line;
  while(std::getline(s, line))
  {
    std::cout << line << '\n';
  }
  return 0;
}
