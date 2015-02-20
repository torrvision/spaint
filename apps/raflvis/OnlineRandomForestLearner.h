/**
 * raflvis: OnlineRandomForestLearner.h
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
 * \brief This class wraps a random forest intended for use as an online learner.
 *
 * Online learning takes place in a sequence of consecutive rounds.
 * On each round, the random forest learner is given a question and is required to provide an answer.
 * For example, a learner might receive a feature from a scene point,
 * and the question is: To which class does the feature belong?.
 * To answer the question, the learner uses a prediction mechanism, termed a hypothesis,
 * which is a mapping from the set of questions (examples) to the set of admissible answers (class labels).
 * After predicting an answer, the learner gets the correct answer to the question.
 * The quality of the learner's answer is assessed by a loss function that measures the discrepancy between the predicted answer and the correct one.
 * The learner's ultimate goal is to minimise the cumulative loss suffered along its run.
 * To achieve this goal, the learner may update the hypothesis after each round so as to be more accurate in later rounds.
 * [Shai Shalev-Shwartz PhD Thesis, Link: ttic.uchicago.edu/~shai/]
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
  mutable RandomForest_Ptr m_randomForest;

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
  /**
   * \brief Gets an answer to the questions posed to the random forest.
   *
   * \param examples      The set of examples.
   * \return              The answer to the questions as quantified by a performance measure.
   */
  std::map<std::string,evaluation::PerformanceMeasure> answer(const std::vector<Example_CPtr>& examples) const
  {
    std::set<Label> classLabels;
    size_t examplesSize = examples.size();
    std::vector<Label> expectedLabels(examplesSize), predictedLabels(examplesSize);

    for(size_t i = 0; i < examplesSize; ++i)
    {
      const Example_CPtr& example = examples[i];
      predictedLabels[i] = predict(example->get_descriptor());
      expectedLabels[i] = example->get_label();
      classLabels.insert(expectedLabels[i]);
    }

    Eigen::MatrixXf confusionMatrix = evaluation::ConfusionMatrixUtil::make_confusion_matrix(classLabels, expectedLabels, predictedLabels);

    return boost::assign::map_list_of("Accuracy", evaluation::ConfusionMatrixUtil::calculate_accuracy(confusionMatrix));
  }

  /*
   * \brief  Predicts a label for thte specified desctiptor.
   *
   * \param desctiptor  The descriptor.
   * return             The predicted label.
   */
  Label predict(Descriptor_CPtr descriptor) const
  {
    return m_randomForest->predict(descriptor);
  }

  /*
   * \brief Ask multiple questions to the random forest posed as a set of examples.
   *
   * \param examples  Pairs of descriptors and labels.
   */
  void question(const std::vector<Example_CPtr>& examples) const
  {
    m_randomForest->add_examples(examples);
  }

  /*
   * \brief Update the random forest to minimize prediction error in the future.
   */
  void update() const
  {
    m_randomForest->train(m_splitBudget);
  }
};

#endif

