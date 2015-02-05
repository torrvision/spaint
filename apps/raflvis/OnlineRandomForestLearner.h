/**
 * raflperf: OnlineRandomForestLearner.h
 */

#ifndef H_RAFLPERF_ONLINERANDOMFORESTLEARNER
#define H_RAFLPERF_ONLINERANDOMFORESTLEARNER

#include <boost/assign/list_of.hpp>
#include <boost/shared_ptr.hpp>

#include <evaluation/core/LearnerEvaluator.h>
#include <evaluation/core/PerformanceMeasure.h>
#include <evaluation/util/ConfusionMatrixUtil.h>

#include <rafl/RandomForest.h>

#include <tvgutil/MapUtil.h>

/**
 * \brief TODO
 */
template <typename Label>
class OnlineRandomForestLearner
{
  //#################### TYPEDEFS AND USINGS ####################
private:
  typedef boost::shared_ptr<const rafl::Example<Label> > Example_CPtr;
  typedef rafl::DecisionTree<Label> DecisionTree;
  typedef rafl::RandomForest<Label> RandomForest;
  typedef boost::shared_ptr<RandomForest> RandomForest_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The settings to use for decision trees in the random forest. */
  typename DecisionTree::Settings m_decisionTreeSettings;

  /** The maximum number of nodes per tree that may be split in each training step. */
  size_t m_splitBudget;

  /** The number of decision trees to use in the random forest. */
  size_t m_treeCount;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an online random forest learner.
   *
   * \param settings        The settings to use for the random forest.
   */
  explicit OnlineRandomForestLearner(const std::map<std::string,std::string>& settings)
  : m_decisionTreeSettings(settings)
  {
    #define GET_SETTING(param) tvgutil::MapUtil::typed_lookup(settings, #param, m_##param);
      GET_SETTING(splitBudget);
      GET_SETTING(treeCount);
    #undef GET_SETTING
  }

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  
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
  static std::map<std::string,evaluation::PerformanceMeasure> do_evaluation(const RandomForest_Ptr& randomForest, const std::vector<Example_CPtr>& examples, const std::vector<size_t>& indices)
  {
    std::set<Label> classLabels;
    size_t indicesSize = indices.size();
    std::vector<Label> expectedLabels(indicesSize), predictedLabels(indicesSize);

    for(size_t i = 0; i < indicesSize; ++i)
    {
      const Example_CPtr& example = examples[indices[i]];
      predictedLabels[i] = randomForest->predict(example->get_descriptor());
      expectedLabels[i] = example->get_label();
      classLabels.insert(expectedLabels[i]);
    }

    Eigen::MatrixXf confusionMatrix = evaluation::ConfusionMatrixUtil::make_confusion_matrix(classLabels, expectedLabels, predictedLabels);
    return boost::assign::map_list_of("Accuracy", evaluation::ConfusionMatrixUtil::calculate_accuracy(confusionMatrix));
  }
};

#endif
