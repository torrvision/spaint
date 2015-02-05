/**
 * raflperf: OnlineRandomForestLearner.h
 */

#ifndef H_RAFLPERF_ONLINERANDOMFORESTLEARNER
#define H_RAFLPERF_ONLINERANDOMFORESTLEARNER

#include <boost/assign/list_of.hpp>
#include <boost/shared_ptr.hpp>

#include <tvgutil/LimitedContainer.h>

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

  /** An instance of a random forest. */
  RandomForest_Ptr m_randomForest;

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

    m_randomForest.reset(new RandomForest(m_treeCount, m_decisionTreeSettings));
  }


  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /*
   * \brief Pass data to the forest.
   *
   * \param examples  TODO.
   */
  void question(const std::vector<Example_CPtr>& examples)
  {
    m_randomForest->add_examples(examples);
  }

  /*
   * \brief Get answers to questions with current model.
   *
   * \param examples  TODO.
   */
  std::map<std::string,evaluation::PerformanceMeasure> answer(const std::vector<Example_CPtr>& examples)
  {
    return do_evaluation(examples);
  }

  /*
   * \brief Update the random forest to minimize prediction error in the future.
   *
   * \param examples  TODO.
   */
  void update()
  {
    m_randomForest->train(m_splitBudget);
  }

  Label predict(Descriptor_CPtr descriptor)
  {
    return m_randomForest->predict(descriptor);
  }

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Evaluates a random forest on a subset of a set of examples.
   *
   * \param examples      The overall set of examples from which the subset of evaluation examples is drawn.
   * \return              The results of the evaluation.
   */
  std::map<std::string,evaluation::PerformanceMeasure> do_evaluation(const std::vector<Example_CPtr>& examples)
  {
    std::set<Label> classLabels;
    size_t examplesSize = examples.size();
    std::vector<Label> expectedLabels(examplesSize), predictedLabels(examplesSize);

    for(size_t i = 0; i < examplesSize; ++i)
    {
      const Example_CPtr& example = examples[i];
      predictedLabels[i] = m_randomForest->predict(example->get_descriptor());
      expectedLabels[i] = example->get_label();
      classLabels.insert(expectedLabels[i]);
      std::cout << "pred=" << predictedLabels[i] << " expected=" << expectedLabels[i] << '\n';
    }

    Eigen::MatrixXf confusionMatrix = evaluation::ConfusionMatrixUtil::make_confusion_matrix(classLabels, expectedLabels, predictedLabels);
    std::cout << confusionMatrix;

    std::map<std::string,evaluation::PerformanceMeasure> performance;
    performance.insert(std::make_pair("Accuracy", evaluation::ConfusionMatrixUtil::calculate_accuracy(confusionMatrix)));
    std::cout << tvgutil::make_limited_container(performance,5) << std::flush;
    return performance;
    //return boost::assign::map_list_of("Accuracy", evaluation::ConfusionMatrixUtil::calculate_accuracy(confusionMatrix));
  }
};

#endif

