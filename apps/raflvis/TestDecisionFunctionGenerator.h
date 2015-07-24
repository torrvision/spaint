/**
 * raflvis: TestDecisionFunctionGenerator.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_RAFLVIS_TESTDECISIONFUNCTIONGENERATOR
#define H_RAFLVIS_TESTDECISIONFUNCTIONGENERATOR

#include <rafl/decisionfunctions/CompositeDecisionFunctionGenerator.h>
#include <rafl/decisionfunctions/FeatureThresholdingDecisionFunctionGenerator.h>
#include <rafl/decisionfunctions/PairwiseOpAndThresholdDecisionFunctionGenerator.h>
using namespace rafl;

/**
 * \brief An instance of this class can be used to test composite decision function generation.
 */
template <typename Label>
class TestDecisionFunctionGenerator : public CompositeDecisionFunctionGenerator<Label>
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<DecisionFunctionGenerator<Label> > DecisionFunctionGenerator_Ptr;
  typedef boost::shared_ptr<const DecisionFunctionGenerator<Label> > DecisionFunctionGenerator_CPtr;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a test decision function generator.
   */
  TestDecisionFunctionGenerator()
  {
    std::pair<int,int> featureIndexRange(0, 1);
    this->add_generator(DecisionFunctionGenerator_CPtr(new FeatureThresholdingDecisionFunctionGenerator<Label>(featureIndexRange)));
    this->add_generator(DecisionFunctionGenerator_CPtr(new PairwiseOpAndThresholdDecisionFunctionGenerator<Label>(featureIndexRange)));
  }

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Makes a test decision function generator.
   *
   * \param params  The parameters to the decision function generator.
   * \return        The decision function generator.
   */
  static DecisionFunctionGenerator_Ptr maker(const std::string& params)
  {
    return DecisionFunctionGenerator_Ptr(new TestDecisionFunctionGenerator<Label>);
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual std::string get_params() const
  {
    return "";
  }

  /**
   * \brief Gets the type of the decision function generator.
   *
   * \return  The type of the decision function generator.
   */
  static std::string get_static_type()
  {
    return "Test";
  }

  /** Override */
  virtual std::string get_type() const
  {
    return get_static_type();
  }
};

#endif
