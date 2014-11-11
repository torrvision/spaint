/**
 * rafl: DecisionFunctionGenerator.h
 */

#ifndef H_RAFL_DECISIONFUNCTIONGENERATOR
#define H_RAFL_DECISIONFUNCTIONGENERATOR

#include <utility>

#include "../base/ProbabilityMassFunction.h"
#include "../examples/Example.h"
#include "DecisionFunction.h"

namespace rafl {

/**
 * \brief An instance of an instantiation of this class template can be used to pick an appropriate decision function to split a set of examples.
 */
template <typename Label>
class DecisionFunctionGenerator
{
  //#################### TYPEDEFS ####################
protected:
  typedef boost::shared_ptr<const Example<Label> > Example_CPtr;

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the generator.
   */
  virtual ~DecisionFunctionGenerator() {}

  //#################### PRIVATE ABSTRACT MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Generates a candidate decision function to split the specified set of examples.
   *
   * \param examples  The examples to split.
   * \return          The candidate decision function.
   */
  virtual DecisionFunction_Ptr generate_candidate(const std::vector<Example_CPtr>& examples) const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Picks an appropriate decision function to split the specified set of examples.
   *
   * \param examples  The examples to split.
   * \return          The chosen decision function.
   */
  DecisionFunction_Ptr pick_decision_function(const std::vector<Example_CPtr>& examples) const
  {
    const int NUM_CANDIDATES = 5;
    for(int i = 0; i < NUM_CANDIDATES; ++i)
    {
      DecisionFunction_Ptr candidate = generate_candidate(examples);
      std::pair<std::vector<Example_CPtr>,std::vector<Example_CPtr> > examplePartition;
      for(size_t j = 0, size = examples.size(); j < size; ++j)
      {
        if(candidate->classify_descriptor(examples[j]->get_descriptor()) == DecisionFunction::DC_LEFT)
        {
          examplePartition.first.push_back(examples[j]);
        }
        else
        {
          examplePartition.second.push_back(examples[j]);
        }
      }
    }
  }

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief TODO
   */
  static ProbabilityMassFunction<Label> make_pmf(const std::vector<Example_CPtr>& examples)
  {
    // TODO
    throw 23;
  }
};

}

#endif
