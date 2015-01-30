/**
 * rafl: DecisionFunctionGeneratorFactory.h
 */

#ifndef H_RAFL_DECISIONFUNCTIONGENERATORFACTORY
#define H_RAFL_DECISIONFUNCTIONGENERATORFACTORY

#include <tvgutil/MapUtil.h>
#include <tvgutil/RandomNumberGenerator.h>

#include "FeatureThresholdingDecisionFunctionGenerator.h"

namespace rafl {

template <typename Label>
class DecisionFunctionGeneratorFactory
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<DecisionFunctionGenerator<Label> > DecisionFunctionGenerator_Ptr;
  typedef std::map<std::string,std::string> Params;
  typedef tvgutil::RandomNumberGenerator_Ptr RandomNumberGenerator_Ptr;

  typedef DecisionFunctionGenerator_Ptr (*Maker)(const Params&, const RandomNumberGenerator_Ptr&);

  //#################### PRIVATE VARIABLES ####################
private:
  std::map<std::string,Maker> m_makers;

  //#################### SINGLETON IMPLEMENTATION ####################
private:
  DecisionFunctionGeneratorFactory()
  {
    m_makers.insert(std::make_pair("FeatureThresholding", &feature_thresholding_maker));
  }

public:
  static DecisionFunctionGeneratorFactory& instance()
  {
    static DecisionFunctionGeneratorFactory s_instance;
    return s_instance;
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  DecisionFunctionGenerator_Ptr make(const std::string& name, const Params& params, const RandomNumberGenerator_Ptr& randomNumberGenerator)
  {
    const Maker& maker = tvgutil::MapUtil::lookup(m_makers, name);
    return (*maker)(params, randomNumberGenerator);
  }

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  static DecisionFunctionGenerator_Ptr feature_thresholding_maker(const Params&, const RandomNumberGenerator_Ptr& randomNumberGenerator)
  {
    return DecisionFunctionGenerator_Ptr(new FeatureThresholdingDecisionFunctionGenerator<Label>(randomNumberGenerator));
  }
};

}

#endif
