/**
 * rafl: ExampleUtil.h
 */

#ifndef H_RAFL_EXAMPLEUTIL
#define H_RAFL_EXAMPLEUTIL

#include <fstream>
#include <string>

#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>
#include <boost/tokenizer.hpp>

#include "../base/ProbabilityMassFunction.h"
#include "Example.h"

namespace rafl {

/**
 * \brief This class contains utility functions for examples.
 */
class ExampleUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Calculates the entropy of the label distribution of a set of examples.
   *
   * \param examples    The examples for whose label distribution we want to calculate the entropy.
   * \param multipliers Optional per-class ratios that can be used to scale the probabilities for the different labels.
   * \return            The entropy of the examples' label distribution.
   */
  template <typename Label>
  static float calculate_entropy(const std::vector<boost::shared_ptr<const Example<Label> > >& examples, const boost::optional<std::map<Label,float> >& multipliers = boost::none)
  {
    return examples.empty() ? 0.0f : make_pmf(examples, multipliers).calculate_entropy();
  }

  /**
   * \brief Calculates the entropy of a label distribution represented by the specified histogram.
   *
   * \param histogram The histogram.
   * \return          The entropy of the label distribution represented by the histogram.
   */
  template <typename Label>
  static float calculate_entropy(const Histogram<Label>& histogram)
  {
    return histogram.empty() ? 0.0f : ProbabilityMassFunction<Label>(histogram).calculate_entropy();
  }

  /**
   * \brief Loads a set of examples from the specified file.
   *
   * \param filename  The name of the file from which to load the examples.
   * \return          The loaded examples.
   */
  template <typename Label>
  static std::vector<boost::shared_ptr<const Example<Label> > > load_examples(const std::string& filename)
  {
    // FIXME: Make this robust to bad data.

    typedef boost::shared_ptr<const Example<Label> > Example_CPtr;
    std::vector<Example_CPtr> result;

    std::ifstream fs(filename.c_str());
    std::string line;
    while(std::getline(fs, line))
    {
      typedef boost::char_separator<char> sep;
      typedef boost::tokenizer<sep> tokenizer;
      tokenizer tok(line.begin(), line.end(), sep(",\r"));
      std::vector<std::string> tokens(tok.begin(), tok.end());

      Descriptor_Ptr descriptor(new Descriptor);
      for(int i = 0; i < tokens.size() - 1; ++i)
      {
        descriptor->push_back(boost::lexical_cast<float>(tokens[i]));
      }

      Label label = boost::lexical_cast<Label>(tokens.back());
      result.push_back(Example_CPtr(new Example<Label>(descriptor, label)));
    }

    return result;
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
   * \param examples    The examples from whose label distribution we want to make a PMF.
   * \param multipliers Optional per-class ratios that can be used to scale the probabilities for the different labels.
   * \return            The PMF.
   */
  template <typename Label>
  static ProbabilityMassFunction<Label> make_pmf(const std::vector<boost::shared_ptr<const Example<Label> > >& examples, const boost::optional<std::map<Label,float> >& multipliers = boost::none)
  {
    return ProbabilityMassFunction<Label>(make_histogram(examples), multipliers);
  }
};

}

#endif
