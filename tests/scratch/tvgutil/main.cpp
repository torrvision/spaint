#include <iostream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

int main()
{
  using boost::property_tree::ptree;
  ptree pt;
  read_xml("C:/foo/foo.txt", pt);
  int candidateCount = pt.get<int>("settings.candidateCount");
  auto x = pt.find("settings.candidateCount");
  std::cout << candidateCount << '\n';
  std::cout << pt.get<int>("blah") << '\n';
  return 0;
}
