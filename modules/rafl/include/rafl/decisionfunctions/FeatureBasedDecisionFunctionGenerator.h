/**
 * rafl: FeatureBasedDecisionFunctionGenerator.h
 */

#ifndef H_RAFL_FEATUREBASEDDECISIONFUNCTIONGENERATOR
#define H_RAFL_FEATUREBASEDDECISIONFUNCTIONGENERATOR

#include <cassert>

#include "DecisionFunctionGenerator.h"

namespace rafl {

/**
 * \brief An instance of an instantiation of a class template deriving from this one can be used to generate a feature-based decision function
 *        with which to split a set of examples.
 *
 * A "feature-based" generator is one that generates functions that base their decisions on the features in the examples' descriptors (as opposed
 * to e.g. sending all examples the same way or basing their decisions on the phase of the moon, etc.)
 */
template <typename Label>
class FeatureBasedDecisionFunctionGenerator : public DecisionFunctionGenerator<Label>
{
  //#################### PRIVATE VARIABLES ####################
private:
  /**
   * An optional (closed) range of indices specifying the features that should be considered when generating decision functions.
   * If no range is specified, all features are considered.
   */
  boost::optional<std::pair<int,int> > m_featureIndexRange;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a decision function generator that can randomly generate feature-based decision functions.
   *
   * \param featureIndexRange An optional (closed) range of indices specifying the features that should be considered when generating decision functions.
   */
  FeatureBasedDecisionFunctionGenerator(const boost::optional<std::pair<int,int> >& featureIndexRange = boost::none)
  : m_featureIndexRange(featureIndexRange)
  {
    if(featureIndexRange && (featureIndexRange->first < 0 || featureIndexRange->first > featureIndexRange->second))
    {
      throw std::runtime_error("Invalid feature index range");
    }
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual std::string get_params() const
  {
    if(m_featureIndexRange)
    {
      return boost::lexical_cast<std::string>(m_featureIndexRange->first) + " " + boost::lexical_cast<std::string>(m_featureIndexRange->second);
    }
    else return "";
  }

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Gets a (closed) range of indices specifying the features that should be considered when generating decision functions.
   *
   * \param descriptorSize  The size of feature descriptor being used by the examples in the set to be split.
   * \return                A (closed) range of indices specifying the features that should be considered when generating decision functions.
   */
  std::pair<int,int> get_feature_index_range(int descriptorSize) const
  {
    std::pair<int,int> result(0, descriptorSize - 1);
    if(m_featureIndexRange)
    {
      result = *m_featureIndexRange;
      assert(result.second < descriptorSize);
    }
    return result;
  }
};

}

#endif
