/**
 * rafl: RFOnlineLearner.h
 */

#ifndef H_RAFL_RFONLINELEARNER
#define H_RAFL_RFONLINELEARNER

#include <map>

#include <boost/spirit/home/support/detail/hold_any.hpp>
using boost::spirit::hold_any;

namespace rafl {

template <typename Result, typename Label>
class RFOnlineLearner
{
  //#################### PUBLIC TYPEDEFS #################### 
public:
  typedef std::map<std::string,hold_any> ParamSet;
  typedef boost::shared_ptr<const Example<Label> > Example_CPtr;
  typedef std::vector<size_t> Indices;
  typedef std::pair<Indices,Indices> Split;

  //#################### PUBLIC MEMBER VARIABLES #################### 
  tvgutil::RandomNumberGenerator m_rng;

  //#################### CONSTRUCTOR #################### 
public:
  explicit RFOnlineLearner(ParamSet settings, unsigned int seed)
  : m_rng(seed)
  {
    //Initialise a random forest!
  }

  Result output(const std::vector<Example_CPtr>& examples, const Split& split)
  {
    return m_rng.generate_real_from_uniform<float>(0.0f, 100.0f);
  }

  //#################### PUBLIC MEMBER FUNCTIONS #################### 
  //#################### PRIVATE MEMBER FUNCTIONS #################### 
};

}

#endif

