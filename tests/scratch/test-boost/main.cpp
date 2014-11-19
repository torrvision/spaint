#include <iostream>
#include <string>

#include <boost/regex.hpp>

int main()
{
  std::string s = "23";
  boost::regex r("\\d+");
  boost::smatch what;
  std::cout << (regex_match(s, what, r) ? "Matched" : "Not matched") << '\n';
  return 0;
}
