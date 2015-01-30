/**
 * rafl: DecisionFunctionGeneratorFactory.h
 */

#ifndef H_RAFL_DECISIONFUNCTIONGENERATORFACTORY
#define H_RAFL_DECISIONFUNCTIONGENERATORFACTORY

#include <tvgutil/MapUtil.h>
#include <tvgutil/RandomNumberGenerator.h>

#include "FeatureThresholdingDecisionFunctionGenerator.h"

namespace rafl {

/**
 * \brief An instance of an instantiation of this class template can be used to make decision function generators.
 */
template <typename Label>
class DecisionFunctionGeneratorFactory
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<DecisionFunctionGenerator<Label> > DecisionFunctionGenerator_Ptr;
  typedef tvgutil::RandomNumberGenerator_Ptr RandomNumberGenerator_Ptr;
  typedef DecisionFunctionGenerator_Ptr (*Maker)(const RandomNumberGenerator_Ptr&);

  //#################### PRIVATE VARIABLES ####################
private:
  /** A map of maker functions for the various different types of decision function generator. */
  std::map<std::string,Maker> m_makers;

  //#################### SINGLETON IMPLEMENTATION ####################
private:
  /**
   * \brief Constructs an instance of the factory.
   */
  DecisionFunctionGeneratorFactory()
  {
    // Register the makers for the various different types of decision function.
    m_makers.insert(std::make_pair("FeatureThresholding", &feature_thresholding_maker));
  }

public:
  /**
   * \brief Gets the unique instance of the factory.
   *
   * \return  The unique instance of the factory.
   */
  static DecisionFunctionGeneratorFactory& instance()
  {
    static DecisionFunctionGeneratorFactory s_instance;
    return s_instance;
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Makes a decision function generator of the specified type.
   *
   * \param type                  The type of decision function generator to make.
   * \param randomNumberGenerator The random number generator needed by certain types of decision function generator.
   * \return                      The decision function generator.
   */
  DecisionFunctionGenerator_Ptr make(const std::string& type, const RandomNumberGenerator_Ptr& randomNumberGenerator)
  {
    const Maker& maker = tvgutil::MapUtil::lookup(m_makers, type);
    return (*maker)(randomNumberGenerator);
  }

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Makes a feature thresholding decision function generator.
   *
   * \param randomNumberGenerator The random number generator needed when generating decision functions.
   * \return                      The decision function generator.
   */
  static DecisionFunctionGenerator_Ptr feature_thresholding_maker(const RandomNumberGenerator_Ptr& randomNumberGenerator)
  {
    return DecisionFunctionGenerator_Ptr(new FeatureThresholdingDecisionFunctionGenerator<Label>(randomNumberGenerator));
  }
};

}

#endif
