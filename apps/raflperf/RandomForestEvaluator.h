/**
 * raflperf: RandomForestEvaluator.h
 */

#ifndef H_RAFLPERF_RANDOMFORESTEVALUATOR
#define H_RAFLPERF_RANDOMFORESTEVALUATOR

#include <boost/assign/list_of.hpp>

#include <rafl/RandomForest.h>
#include <rafl/evaluation/AlgorithmEvaluator.h>
#include <rafl/evaluation/PerformanceMeasure.h>

#include <tvgutil/MapUtil.h>

/**
 * \brief TODO
 */
template <typename Label>
class RandomForestEvaluator : public rafl::AlgorithmEvaluator<rafl::Example<Label>,std::map<std::string,PerformanceMeasure> >
{
  //#################### TYPEDEFS AND USINGS ####################
private:
  typedef rafl::AlgorithmEvaluator<rafl::Example<Label>,std::map<std::string,PerformanceMeasure> > Base;
  typedef rafl::DecisionTree<Label> DecisionTree;
  using typename Base::Example_CPtr;
  typedef rafl::RandomForest<Label> RandomForest;
  typedef boost::shared_ptr<RandomForest> RandomForest_Ptr;
  using typename Base::ResultType;

  //#################### PRIVATE VARIABLES ####################
private:
  typename DecisionTree::Settings m_decisionTreeSettings;
  size_t m_splitBudget;
  size_t m_treeCount;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief TODO
   */
  explicit RandomForestEvaluator(const rafl::SplitGenerator_Ptr& splitGenerator, const std::map<std::string,std::string>& settings)
  : Base(splitGenerator), m_decisionTreeSettings(settings)
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
  virtual ResultType evaluate_on_split(const std::vector<Example_CPtr>& examples, const SplitGenerator::Split& split) const
  {
    RandomForest_Ptr randomForest(new RandomForest(m_treeCount, m_decisionTreeSettings));

    // Add the training examples to the forest.
    randomForest->add_examples(examples, split.first);

    // Train the forest.
    randomForest->train(m_splitBudget);

    // Evaluate the forest on the validation set.
    return do_evaluation(randomForest, examples, split.second);
  }

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Evaluates a random forest on a set of examples.
   *
   * \param randomForest  The random forest.
   * \param examples      The set of examples on which to evaluate it.
   * \param indices       The indices of the examples to use in the evaluation.
   * \return              The results of the evaluation.
   */
  ResultType do_evaluation(const RandomForest_Ptr& randomForest, const std::vector<Example_CPtr>& examples, const std::vector<size_t>& indices) const
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

    return boost::assign::map_list_of("Accuracy", PerfUtil::get_accuracy(PerfUtil::get_confusion_matrix(classLabels, expectedLabels, predictedLabels)));
  }
};

#endif
