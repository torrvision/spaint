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

#include <rafl/evaluation/PerformanceMeasureSet.h>
#include <rafl/evaluation/PerfUtil.h>

namespace rafl {

/**
 * \brief This class is a wrapper around a random forest and provides tools for online learning and evaluation.
 */
template <typename Label>
class RFOnlineLearner
{
  //#################### PUBLIC TYPEDEFS ####################
public:
  typedef boost::shared_ptr<const Example<Label> > Example_CPtr;
  typedef std::vector<size_t> Indices;
  typedef std::pair<Indices,Indices> Split;
  typedef DecisionTree<Label> DT; 
  typedef RandomForest<Label> RF;
  typedef PerformanceMeasureSet Result;

  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  typename DT::Settings m_decisionTreeSettings;

  /** The split budget of the random forest which may change over time. */
  size_t m_splitBudget;

  size_t m_treeCount;

  //#################### CONSTRUCTOR ####################
public:
  /**
   * \brief Constructs a random forest with particular parameter settings.
   *
   * \param The settings of the random forest.
   */
  explicit RFOnlineLearner(const std::map<std::string,std::string>& settings)
  : m_decisionTreeSettings(settings)
  {
    #define GET_SETTING(param) tvgutil::MapUtil::typed_lookup(settings, #param, m_##param);
      GET_SETTING(splitBudget); //this is the initial split budget as it may change over time.
      GET_SETTING(treeCount);
    #undef GET_SETTING
  }

  /**
   * \brief Given an offline training set,
   * this function trains the random forest on the examples selected by the first split,
   * and evaluates the random forest on the examples selected by the second split.
   */
  PerformanceMeasureSet evaluate_on_split(const std::vector<Example_CPtr>& examples, const Split& split) const
  {
    boost::shared_ptr<RF> randomForest(new RF(m_treeCount, m_decisionTreeSettings));

    //Add training examples to forest.
    randomForest->add_examples(examples, split.first);

    //Train the forest.
    randomForest->train(m_splitBudget);

    //Predict on the validation set.
    return evaluate(randomForest, examples, split.second);
  }

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief This function evaluates the random forest on a set of examples.
   *
   * \param randomForest  The random forest.
   * \param examples      The set of examples to evaluate.
   * \param incides       The indices of the examples to use in the evaluation.
   * \return              The quantitative performance measures.
   */
  PerformanceMeasureSet evaluate(const boost::shared_ptr<RF>& randomForest, const std::vector<Example_CPtr>& examples, const std::vector<size_t>& indices) const
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
      predictedLabels[i] = randomForest->predict(descriptor);
    }

    // Calculates the quantitative performance measures.
    std::map<std::string,PerformanceMeasure> measures;
    PerformanceMeasure measure("Accuracy", PerfUtil::get_accuracy(PerfUtil::get_confusion_matrix(classLabels, expectedLabels, predictedLabels)));
    measures.insert(std::make_pair(measure.get_name(), measure));
    return PerformanceMeasureSet(measures);
  }
};

}

#endif
