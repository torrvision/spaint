/**
 * rafl: DecisionFunctionGenerator.h
 */

#ifndef H_RAFL_DECISIONFUNCTIONGENERATOR
#define H_RAFL_DECISIONFUNCTIONGENERATOR

#include "DecisionFunction.h"
#include "../examples/Example.h"

namespace rafl {

/**
 * \brief An instance of an instantiation of this class template can be used to pick an appropriate decision function to split a set of examples.
 */
template <typename Label>
class DecisionFunctionGenerator
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const Example<Label> > Example_CPtr;

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the generator.
   */
  virtual ~DecisionFunctionGenerator() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Picks an appropriate decision function to split the specified set of examples.
   *
   * \param examples  The examples to split.
   * \return          The chosen decision function.
   */
  virtual DecisionFunction_Ptr pick_decision_function(const std::vector<Example_CPtr>& examples) const = 0;
};

}

#endif
