/**
 * rafl: FeatureBasedDecisionFunctionGenerator.h
 */

#ifndef H_RAFL_FEATUREBASEDDECISIONFUNCTIONGENERATOR
#define H_RAFL_FEATUREBASEDDECISIONFUNCTIONGENERATOR

#include "DecisionFunctionGenerator.h"

namespace rafl {

/**
 * \brief TODO
 */
template <typename Label>
class FeatureBasedDecisionFunctionGenerator : public DecisionFunctionGenerator<Label>
{
  //#################### PRIVATE VARIABLES ####################
private:
  /**
   * An optional range of indices specifying the features that should be considered when generating decision functions.
   * If no range is specified, all features are considered.
   */
  boost::optional<std::pair<size_t,size_t> > m_featureIndexRange;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a decision function generator that can randomly generate feature-based decision functions.
   *
   * \param featureIndexRange An optional range of indices specifying the features that should be considered when generating decision functions.
   */
  FeatureBasedDecisionFunctionGenerator(const boost::optional<std::pair<size_t,size_t> >& featureIndexRange = boost::none)
  : m_featureIndexRange(featureIndexRange)
  {}

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief TODO
   *
   * \param descriptorSize  TODO
   * \return                TODO
   */
  std::pair<int,int> get_feature_index_range(int descriptorSize) const
  {
    std::pair<int,int> result(0, descriptorSize);
    if(m_featureIndexRange)
    {
      result.first = static_cast<int>(m_featureIndexRange->first);
      result.second = static_cast<int>(m_featureIndexRange->second);
      if(result.second < result.first || result.second >= descriptorSize) throw std::runtime_error("Invalid feature index range");
    }
    return result;
  }
};

}

#endif
