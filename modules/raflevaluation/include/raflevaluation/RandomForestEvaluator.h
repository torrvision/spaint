/**
 * raflevaluation: RandomForestEvaluator.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_RAFLEVALUATION_RANDOMFORESTEVALUATOR
#define H_RAFLEVALUATION_RANDOMFORESTEVALUATOR

#include <boost/assign/list_of.hpp>

#include <evaluation/core/LearnerEvaluator.h>
#include <evaluation/core/PerformanceMeasure.h>
#include <evaluation/util/ConfusionMatrixUtil.h>

#include <rafl/core/RandomForest.h>

#include <tvgutil/containers/MapUtil.h>

namespace raflevaluation {

/**
 * \brief An instance of this class can be used to evaluate a random forest using approaches based on example set splitting.
 */
template <typename Label>
class RandomForestEvaluator : public evaluation::LearnerEvaluator<rafl::Example<Label>,std::map<std::string,PerformanceMeasure> >
{
  //#################### TYPEDEFS AND USINGS ####################
private:
  typedef evaluation::LearnerEvaluator<rafl::Example<Label>,std::map<std::string,PerformanceMeasure> > Base;
  using typename Base::Example_CPtr;
  using typename Base::ResultType;
  typedef rafl::DecisionTree<Label> DecisionTree;
  typedef rafl::RandomForest<Label> RandomForest;
  typedef boost::shared_ptr<RandomForest> RandomForest_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The settings to use for the random forest. */
  std::map<std::string,std::string> m_settings;

  /** The maximum number of nodes per tree that may be split in each training step. */
  size_t m_splitBudget;

  /** The number of decision trees to use in the random forest. */
  size_t m_treeCount;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a random forest evaluator.
   *
   * \param splitGenerator  The generator to use to split the example set.
   * \param settings        The settings to use for the random forest.
   */
  explicit RandomForestEvaluator(const evaluation::SplitGenerator_Ptr& splitGenerator, const std::map<std::string,std::string>& settings)
  : Base(splitGenerator), m_settings(settings)
  {
    #define GET_SETTING(param) tvgutil::MapUtil::typed_lookup(settings, #param, m_##param);
      GET_SETTING(splitBudget);
      GET_SETTING(treeCount);
    #undef GET_SETTING
  }

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /** Override */
  virtual std::map<std::string,PerformanceMeasure> average_results(const std::vector<std::map<std::string,PerformanceMeasure> >& results) const
  {
    std::map<std::string,std::vector<PerformanceMeasure> > groupedResults;

    // Group the results by measure.
    for(typename std::vector<std::map<std::string,PerformanceMeasure> >::const_iterator it = results.begin(), iend = results.end(); it != iend; ++it)
    {
      const std::map<std::string,PerformanceMeasure>& result = *it;
      for(typename std::map<std::string,PerformanceMeasure>::const_iterator jt = result.begin(), jend = result.end(); jt != jend; ++jt)
      {
        groupedResults[jt->first].push_back(jt->second);
      }
    }

    // Average the results for each measure.
    std::map<std::string,PerformanceMeasure> averagedResults;
    for(std::map<std::string,std::vector<PerformanceMeasure> >::const_iterator it = groupedResults.begin(), iend = groupedResults.end(); it != iend; ++it)
    {
      averagedResults.insert(std::make_pair(it->first, PerformanceMeasure::average(it->second)));
    }

    return averagedResults;
  }

  /** Override */
  virtual ResultType evaluate_on_split(const std::vector<Example_CPtr>& examples, const evaluation::SplitGenerator::Split& split) const
  {
    // Make a random forest using the specified settings and add the examples in the training set to it.
    RandomForest_Ptr randomForest(new RandomForest(m_treeCount, typename DecisionTree::Settings(m_settings)));
    randomForest->add_examples(examples, split.first);

    // Train the forest.
    randomForest->train(m_splitBudget);

    // Return the results of evaluating the forest on the validation set.
    return do_evaluation(randomForest, examples, split.second);
  }

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Evaluates a random forest on a subset of a set of examples.
   *
   * \param randomForest  The random forest.
   * \param examples      The overall set of examples from which the subset of evaluation examples is drawn.
   * \param indices       The indices of the subset of examples on which to evaluate the random forest.
   * \return              The results of the evaluation.
   */
  static ResultType do_evaluation(const RandomForest_Ptr& randomForest, const std::vector<Example_CPtr>& examples, const std::vector<size_t>& indices)
  {
    std::set<Label> classLabels;
    int indicesSize = static_cast<int>(indices.size());
    std::vector<Label> expectedLabels(indicesSize), predictedLabels(indicesSize);

#ifdef WITH_OPENMP
    #pragma omp parallel for
#endif
    for(int i = 0; i < indicesSize; ++i)
    {
      const Example_CPtr& example = examples[indices[i]];
      predictedLabels[i] = randomForest->predict(example->get_descriptor());
      expectedLabels[i] = example->get_label();

#ifdef WITH_OPENMP
      #pragma omp critical
#endif
      classLabels.insert(expectedLabels[i]);
    }

    Eigen::MatrixXf confusionMatrix = ConfusionMatrixUtil::make_confusion_matrix(classLabels, expectedLabels, predictedLabels);
    return boost::assign::map_list_of("Accuracy", ConfusionMatrixUtil::calculate_accuracy(ConfusionMatrixUtil::normalise_rows_L1(confusionMatrix)));
  }
};

}

#endif
