/**
 * rafl: FeatureBasedDecisionFunctionGenerator.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_RAFL_FEATUREBASEDDECISIONFUNCTIONGENERATOR
#define H_RAFL_FEATUREBASEDDECISIONFUNCTIONGENERATOR

#include <cassert>
#include <utility>

#include "DecisionFunctionGenerator.h"

#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>

namespace rafl {

/**
 * \brief An instance of an instantiation of a class template deriving from this one can be used to generate a feature-based decision function
 *        with which to split a set of examples.
 *
 * A "feature-based" decision function is one that bases its decisions on the features in the examples' descriptors (as opposed to e.g. sending
 * all examples the same way or basing its decisions on the phase of the moon, etc.)
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

  /**
   * \brief Parses a string specifying the parameters to a feature-based decision function generator.
   *
   * \param params  A string specifying the parameters to a feature-based decision function generator.
   * \return        The parsed parameters.
   */
  static boost::optional<std::pair<int,int> > parse_params(const std::string& params)
  {
    std::string trimmedParams = params;
    boost::algorithm::trim(trimmedParams);
    if(trimmedParams.empty()) return boost::none;

    boost::regex expression("(\\d+)\\s+(\\d+)");
    boost::smatch what;
    if(boost::regex_match(trimmedParams, what, expression))
    {
      int lower = boost::lexical_cast<int>(what[1]);
      int upper = boost::lexical_cast<int>(what[2]);
      return std::make_pair(lower, upper);
    }
    else throw std::runtime_error("The parameters supplied for the feature-based decision function generator are not in the expected format");
  }
};

}

#endif
