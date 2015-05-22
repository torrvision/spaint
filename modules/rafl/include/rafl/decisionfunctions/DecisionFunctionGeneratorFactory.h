/**
 * rafl: DecisionFunctionGeneratorFactory.h
 */

#ifndef H_RAFL_DECISIONFUNCTIONGENERATORFACTORY
#define H_RAFL_DECISIONFUNCTIONGENERATORFACTORY

#include <tvgutil/MapUtil.h>

#include "FeatureThresholdingDecisionFunctionGenerator.h"
#include "PairwiseOpAndThresholdDecisionFunctionGenerator.h"

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
  typedef DecisionFunctionGenerator_Ptr (*Maker)();

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
    // Register the makers for the various different types of decision function generator.
    register_maker(FeatureThresholdingDecisionFunctionGenerator<Label>::get_static_type(), &feature_thresholding_maker);
    register_maker(PairwiseOpAndThresholdDecisionFunctionGenerator<Label>::get_static_type(), &pairwise_op_and_threshold_maker);
  }

public:
  /**
   * \brief Gets the singleton instance.
   *
   * \return  The singleton instance.
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
   * \param type  The type of decision function generator to make.
   * \return      The decision function generator.
   */
  DecisionFunctionGenerator_Ptr make(const std::string& type)
  {
    const Maker& maker = tvgutil::MapUtil::lookup(m_makers, type);
    return (*maker)();
  }

  /**
   * \brief Registers a maker function for a particular type of decision function generator.
   *
   * \param generatorType The type of decision function generator.
   * \param maker         The maker function.
   */
  void register_maker(const std::string& generatorType, Maker maker)
  {
    m_makers.insert(std::make_pair(generatorType, maker));
  }

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Makes a feature thresholding decision function generator.
   *
   * \return  The decision function generator.
   */
  static DecisionFunctionGenerator_Ptr feature_thresholding_maker()
  {
    return DecisionFunctionGenerator_Ptr(new FeatureThresholdingDecisionFunctionGenerator<Label>);
  }

  /**
   * \brief Makes a pairwise operation and thresholding decision function generator.
   *
   * \return  The decision function generator.
   */
  static DecisionFunctionGenerator_Ptr pairwise_op_and_threshold_maker()
  {
    return DecisionFunctionGenerator_Ptr(new PairwiseOpAndThresholdDecisionFunctionGenerator<Label>);
  }
};

}

#endif
