/**
 * raflvis: SpecialDecisionFunctionGenerator.h
 */

#ifndef H_RAFLVIS_SPECIALDECISIONFUNCTIONGENERATOR
#define H_RAFLVIS_SPECIALDECISIONFUNCTIONGENERATOR

#include <rafl/decisionfunctions/CompositeDecisionFunctionGenerator.h>
#include <rafl/decisionfunctions/FeatureThresholdingDecisionFunctionGenerator.h>
using namespace rafl;

/**
 * \brief An instance of this class can be used to generate a decision function with which to split a set of examples.
 */
template <typename Label>
class SpecialDecisionFunctionGenerator : public CompositeDecisionFunctionGenerator<Label>
{
  //#################### TYPEDEFS #################### 
private:
  typedef boost::shared_ptr<DecisionFunctionGenerator<Label> > DecisionFunctionGenerator_Ptr;

  //#################### CONSTRUCTORS #################### 
public:
  /**
   * \brief Constructs a special decision function generator.
   */
  SpecialDecisionFunctionGenerator()
  {
    std::pair<int,int> specialFeatureIndexRange(0, 0);
    this->add_generator(DecisionFunctionGenerator_CPtr(new FeatureThresholdingDecisionFunctionGenerator<Label>(specialFeatureIndexRange)));
  }

  //#################### PUBLIC STATIC MEMBER FUNCTIONS #################### 
public:
  /**
   * \brief Makes a special decision function generator.
   *
   * \return  The decision function generator.
   */
  static DecisionFunctionGenerator_Ptr maker()
  {
    return DecisionFunctionGenerator_Ptr(new SpecialDecisionFunctionGenerator<Label>);
  }

  //#################### PUBLIC MEMBER FUNCTIONS #################### 
public:
  /**
   * \brief Gets the type of the decision function generator.
   * 
   * \return  The type of the decision function generator.
   */
  static std::string get_static_type()
  {
    return "Special";
  }

  /** Override */
  virtual std::string get_type() const
  {
    return get_static_type();
  }
};

#endif
