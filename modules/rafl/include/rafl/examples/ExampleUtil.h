/**
 * rafl: ExampleUtil.h
 */

#ifndef H_RAFL_EXAMPLEUTIL
#define H_RAFL_EXAMPLEUTIL

#include "../base/ProbabilityMassFunction.h"
#include "Example.h"

namespace rafl {

/**
 * \brief This class contains utility functions that work on examples.
 */
class ExampleUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Calculates the entropy of the label distribution of a set of examples.
   *
   * \param examples  The examples for whose label distribution we want to calculate the entropy.
   * \return          The entropy of the examples' label distribution.
   */
  template <typename Label>
  static float calculate_entropy(const std::vector<boost::shared_ptr<const Example<Label> > >& examples)
  {
    return make_pmf(examples).calculate_entropy();
  }

  /**
   * \brief Makes a histogram from the label distribution of a set of examples.
   *
   * \param examples  The examples from whose label distribution we want to make a histogram.
   * \return          The histogram.
   */
  template <typename Label>
  static Histogram<Label> make_histogram(const std::vector<boost::shared_ptr<const Example<Label> > >& examples)
  {
    typedef boost::shared_ptr<const Example<Label> > Example_CPtr;

    Histogram<Label> histogram;
    for(typename std::vector<Example_CPtr>::const_iterator it = examples.begin(), iend = examples.end(); it != iend; ++it)
    {
      histogram.add((*it)->get_label());
    }

    return histogram;
  }

  /**
   * \brief Makes a probability mass function (PMF) from the label distribution of a set of examples.
   *
   * \param examples  The examples from whose label distribution we want to make a PMF.
   * \return          The PMF.
   */
  template <typename Label>
  static ProbabilityMassFunction<Label> make_pmf(const std::vector<boost::shared_ptr<const Example<Label> > >& examples)
  {
    return ProbabilityMassFunction<Label>(make_histogram(examples));
  }
};

}

#endif
