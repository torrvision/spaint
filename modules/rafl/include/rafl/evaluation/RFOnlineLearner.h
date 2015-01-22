/**
 * rafl: RFOnlineLearner.h
 */

#ifndef H_RAFL_RFONLINELEARNER
#define H_RAFL_RFONLINELEARNER

#include <cmath>
#include <map>
#include <utility>

#include <boost/spirit/home/support/detail/hold_any.hpp>

#include <rafl/RandomForest.h>

#include "PerfUtil.h"
#include "QuantitativePerformance.h"

namespace rafl {
  
/**
 * \brief This class is a wrapper around a random forest and provides tools for online learning and evaluation. 
 */
template <typename Label>
class RFOnlineLearner
{
  //#################### PUBLIC TYPEDEFS #################### 
public:
  typedef std::map<std::string,boost::spirit::hold_any> ParamSet;
  typedef boost::shared_ptr<const Example<Label> > Example_CPtr;
  typedef std::vector<size_t> Indices;
  typedef std::pair<Indices,Indices> Split;
  typedef DecisionTree<Label> DT; typedef RandomForest<Label> RF;

  //#################### PRIVATE MEMBER VARIABLES #################### 
private:
  /** An instance of a random forest. */
  RF m_randomForest;

  /** The split budget of the random forest which may change over time. */
  size_t m_splitBudget;

  //#################### CONSTRUCTOR #################### 
public:
  /**
   * \brief Constructs a random forest with particular parameter settings.
   *
   * \param The settings of the random forest.
   */
  explicit RFOnlineLearner(const ParamSet& settings)
  : m_randomForest(settings)
  {
    size_t splitBudget = 0; //this is the initial split budget as it may change over time.
    #define GET_SETTING(param) DT::Settings::set_from_paramset(settings, param, #param);
      GET_SETTING(splitBudget);
    #undef GET_SETTING
    m_splitBudget = splitBudget;
  }

  /**
   * \brief Given an offline training set, 
   * this function trains the random forest on the examples selected by the first split, 
   * and evaluates the random forest on the examples selected by the second split.
   */
  QuantitativePerformance cross_validation_offline_output(const std::vector<Example_CPtr>& examples, const Split& split)
  {
    //Add training examples to forest.
    m_randomForest.add_examples(examples, split.first);

    //Train the forest.
    m_randomForest.train(m_splitBudget);

    //Predict on the validation set.
    return evaluate(examples, split.second);
  }

  //#################### PRIVATE MEMBER FUNCTIONS #################### 
private:
  /**
   * \brief This function evaluates the random forest on a set of examples. 
   *
   * \param examples  The set of examples to evaluate.
   * \param incides   The indices of the examples to use in the evaluation.
   * \return          The quantitative performance. 
   */
  QuantitativePerformance evaluate(const std::vector<Example_CPtr>& examples, const std::vector<size_t>& indices)
  {
    size_t indicesSize = indices.size();
    std::set<Label> classLabels;
    std::vector<Label> expectedLabels(indicesSize), predictedLabels(indicesSize);
    for(size_t i = 0; i < indicesSize; ++i)
    {
      const Example_CPtr example = examples[indices[i]];
      const Descriptor_CPtr& descriptor = example->get_descriptor();
      expectedLabels[i] = example->get_label();
      classLabels.insert(expectedLabels[i]);
      predictedLabels[i] = m_randomForest.predict(descriptor);
    }
    
    //Calculates the quantitative performance measures.
    QuantitativePerformance accuracy(PerfUtil::get_accuracy(PerfUtil::get_confusion_matrix(classLabels, expectedLabels, predictedLabels)));
    return accuracy;
  }
};

}

#endif

