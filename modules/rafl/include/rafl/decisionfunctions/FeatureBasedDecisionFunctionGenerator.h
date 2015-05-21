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
  boost::optional<std::pair<int,int> > m_featureIndexRange;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a decision function generator that can randomly generate feature-based decision functions.
   *
   * \param featureIndexRange An optional range of indices specifying the features that should be considered when generating decision functions.
   */
  FeatureBasedDecisionFunctionGenerator(const boost::optional<std::pair<int,int> >& featureIndexRange = boost::none)
  : m_featureIndexRange(featureIndexRange)
  {
    if(featureIndexRange && (featureIndexRange->first < 0 || featureIndexRange->first > featureIndexRange->second))
    {
      throw std::runtime_error("Invalid feature index range");
    }
  }

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
    std::pair<int,int> result(0, descriptorSize - 1);
    if(m_featureIndexRange)
    {
      result = *m_featureIndexRange;
      if(result.second >= descriptorSize) throw std::runtime_error("Invalid feature index range");
    }
    return result;
  }
};

}

#endif
