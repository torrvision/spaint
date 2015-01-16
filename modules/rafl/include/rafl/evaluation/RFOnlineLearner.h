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

#include "PerformanceEvaluation.h"

namespace rafl {
  
class Result
{
private:
  std::pair<float,float> m_accuracy;
  size_t m_samples;

public:
  explicit Result(float accuracy)
  : m_accuracy(std::make_pair(accuracy, 0.0f)), m_samples(1)
  {
  }
  explicit Result(const std::vector<Result> results)
  : m_samples(results.size())
  {
    //return std::accumulate(results.begin(), results.end(), 0.0)/results.size();
    float sumMean = 0;
    for(size_t i = 0; i < m_samples; ++i)
    {
      sumMean += results[i].mean_accuracy();
    }
    float mean = sumMean/m_samples;
    m_accuracy.first = mean;

    float sumVariance = 0;
    for(size_t i = 0; i < m_samples; ++i)
    {
      sumVariance += pow(mean - results[i].mean_accuracy(), 2);
    }
    m_accuracy.second = sqrt(sumVariance/m_samples);
  }

  float mean_accuracy() const
  {
    return m_accuracy.first;
  }

  friend std::ostream& operator<<(std::ostream& out, const Result& result)
  {
    out << "accuracy: " << result.m_accuracy.first << " +/- " << result.m_accuracy.second << ", samples: " << result.m_samples << "\n";
    return out;
  }
};


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

  //#################### PUBLIC MEMBER VARIABLES #################### 

  //#################### PRIVATE MEMBER VARIABLES #################### 
private:
  /** An instance of a random forest. */
  RF m_randomForest;

  /** The split budget of the random forest which may change over time. */
  size_t m_splitBudget;

  //#################### CONSTRUCTOR #################### 
public:
  explicit RFOnlineLearner(const ParamSet& settings)
  : m_randomForest(settings)
  {
    size_t splitBudget = 0; //this is the initial slit budget as it may change over time.
    #define GET_SETTING(param) DT::Settings::get_from_param_set(settings, param, #param);
      GET_SETTING(splitBudget);
    #undef GET_SETTING
    m_splitBudget = splitBudget;
  }

  Result cross_validation_offline_output(const std::vector<Example_CPtr>& examples, const Split& split)
  {
    //Add training examples to forest.
    m_randomForest.add_examples(examples, split.first);

    //Train the forest.
    m_randomForest.train(m_splitBudget);

    //Predict on the validation set.
    return evaluate(examples, split.second);
    //return m_rng.generate_real_from_uniform<float>(0.0f, 100.0f);
  }

  //#################### PUBLIC MEMBER FUNCTIONS #################### 
  //#################### PRIVATE MEMBER FUNCTIONS #################### 
private:
  Result evaluate(const std::vector<Example_CPtr>& examples, const std::vector<size_t>& indices)
  {
    size_t indicesSize = indices.size();
    std::set<Label> classLabels;
    std::vector<Label> expectedLabels(indicesSize), predictedLabels(indicesSize);
    for(size_t i = 0; i < indicesSize; ++i)
    {
      const Example_CPtr example = examples.at(indices[i]);
      const Descriptor_CPtr& descriptor = example->get_descriptor();
      expectedLabels[i] = example->get_label();
      classLabels.insert(expectedLabels[i]);
      predictedLabels[i] = m_randomForest.predict(descriptor);
    }
    
    Result accuracy(PerfEval::get_accuracy(PerfEval::get_conf_mtx(classLabels, expectedLabels, predictedLabels)));
    return accuracy;
  }
};

}

#endif

