#ifndef H_RAFL_PARAMETERSTRINGGENERATOR
#define H_RAFL_PARAMETERSTRINGGENERATOR

#include <algorithm>
#include <iostream>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

#include <boost/assign.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/spirit/home/support/detail/hold_any.hpp>
using boost::assign::list_of;
using boost::spirit::hold_any;

namespace rafl {

class ParameterStringGenerator
{
private:
std::vector<std::pair<std::string,std::vector<hold_any> > > m_paramOptions;

public:
  ParameterStringGenerator& add_param(const std::string& param, const std::vector<hold_any>& options)
  {
    m_paramOptions.push_back(std::make_pair(param, options));
    return *this;
  }

  std::vector<std::string> generate() const
  {
    return generate_strings_for_params(0);
  }

private:
  static std::vector<std::string> generate_strings_for_param(const std::string& param, const std::vector<hold_any>& options)
  {
    std::vector<std::string> result;
    for(std::vector<hold_any>::const_iterator it = options.begin(), iend = options.end(); it != iend; ++it)
    {
      result.push_back(param + " " + boost::lexical_cast<std::string>(*it));
    }
    return result;
  }

  std::vector<std::string> generate_strings_for_params(size_t from) const
  {
    if(from == m_paramOptions.size())
    {
      return list_of<std::string>("");
    }
    else
    {
      std::vector<std::string> lhs = generate_strings_for_param(m_paramOptions[from].first, m_paramOptions[from].second);
      std::vector<std::string> rhs = generate_strings_for_params(from + 1);
      std::vector<std::string> result;
      for(size_t i = 0, isize = lhs.size(); i < isize; ++i)
      {
        for(size_t j = 0, jsize = rhs.size(); j < jsize; ++j)
        {
          result.push_back(lhs[i] + " " + rhs[j]);
        }
      }
      return result;
    }
  }
};

}

#endif

/*int main()
{
  std::vector<std::string> paramStrings = ParameterStringGenerator()
    .add_param("-n", list_of(3)(4)(5))
    .add_param("-t", list_of<std::string>("Foo")("Bar"))
    .add_param("-x", list_of(23.0f)(9.0f))
    .generate();
  std::copy(paramStrings.begin(), paramStrings.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
  return 0;
}*/
