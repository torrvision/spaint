/**
 * raflperf: RandomForestEvaluator.h
 */

#ifndef H_RAFLPERF_RANDOMFORESTEVALUATOR
#define H_RAFLPERF_RANDOMFORESTEVALUATOR

#include <rafl/evaluation/AlgorithmEvaluator.h>
#include <rafl/evaluation/PerformanceMeasure.h>
#include <rafl/examples/Example.h>

/**
 * \brief TODO
 */
template <typename Label>
class RandomForestEvaluator : public rafl::AlgorithmEvaluator<rafl::Example<Label>,std::map<std::string,PerformanceMeasure> >
{
  //#################### TYPEDEFS ####################
private:
  typedef rafl::AlgorithmEvaluator<rafl::Example<Label>,std::map<std::string,PerformanceMeasure> > Base;
  using typename Base::Example_CPtr;
  using typename Base::ResultType;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief TODO
   */
  explicit RandomForestEvaluator(const rafl::SplitGenerator_Ptr& splitGenerator, const std::map<std::string,std::string>& params)
  : Base(splitGenerator)
  {}

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /** Override */
  virtual ResultType average_results(const std::vector<ResultType>& results) const
  {
    // TODO
    throw 23;
  }

  /** Override */
  virtual ResultType evaluate_on_split(const std::vector<Example_CPtr>& examples, const SplitGenerator::Split& split) const
  {
    // TODO
    throw 23;
  }
};

#endif
