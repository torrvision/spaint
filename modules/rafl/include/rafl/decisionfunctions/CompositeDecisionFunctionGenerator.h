/**
 * rafl: CompositeDecisionFunctionGenerator.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_RAFL_COMPOSITEDECISIONFUNCTIONGENERATOR
#define H_RAFL_COMPOSITEDECISIONFUNCTIONGENERATOR

#include "DecisionFunctionGenerator.h"

namespace rafl {

/**
 * \brief An instance of an instantiation of a class template deriving from this one can be used to generate a decision function with which
 *        to split a set of examples, using one of a number of different generators (picked at random).
 */
template <typename Label>
class CompositeDecisionFunctionGenerator : public DecisionFunctionGenerator<Label>
{
  //#################### TYPEDEFS ####################
protected:
  typedef boost::shared_ptr<const DecisionFunctionGenerator<Label> > DecisionFunctionGenerator_CPtr;

  //#################### USINGS ####################
protected:
  using typename DecisionFunctionGenerator<Label>::Example_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** An array of subsidiary generators that can be used to generate candidate decision functions. */
  std::vector<DecisionFunctionGenerator_CPtr> m_generators;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual DecisionFunction_Ptr generate_candidate_decision_function(const std::vector<Example_CPtr>& examples, const tvgutil::RandomNumberGenerator_Ptr& randomNumberGenerator) const
  {
    // Pick a random subsidiary generator and use it to generate a candidate decision function.
    int generatorIndex = randomNumberGenerator->generate_int_from_uniform(0, static_cast<int>(m_generators.size()) - 1);
    return m_generators[generatorIndex]->generate_candidate_decision_function(examples, randomNumberGenerator);
  }

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Adds a subsidiary generator to the composite.
   *
   * \param generator The generator to add.
   */
  void add_generator(const DecisionFunctionGenerator_CPtr& generator)
  {
    m_generators.push_back(generator);
  }
};

}

#endif
