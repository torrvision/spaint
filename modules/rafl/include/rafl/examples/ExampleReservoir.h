/**
 * rafl: ExampleReservoir.h
 */

#ifndef H_RAFL_EXAMPLERESERVOIR
#define H_RAFL_EXAMPLERESERVOIR

#include <algorithm>
#include <cassert>
#include <iosfwd>
#include <iterator>
#include <map>
#include <vector>

#include <tvgutil/RandomNumberGenerator.h>

#include "../base/Histogram.h"
#include "Example.h"

namespace rafl {

/**
 * \brief An instance of an instantiation of this class template represents a reservoir to store the examples for a node.
 */
template <typename Label>
class ExampleReservoir
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const Example<Label> > Example_CPtr;
  typedef boost::shared_ptr<Histogram<Label> > Histogram_Ptr;
  typedef boost::shared_ptr<const Histogram<Label> > Histogram_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The total number of examples currently in the reservoir. */
  size_t m_curSize;

  /** The examples in the reservoir. */
  std::map<Label,std::vector<Example_CPtr> > m_examples;

  /** The histogram of the label distribution of all of the examples that have ever been added to the reservoir. */
  Histogram_Ptr m_histogram;

  /** The maximum number of examples of each class allowed in the reservoir at any one time. */
  size_t m_maxClassSize;

  /** A random number generator. */
  tvgutil::RandomNumberGenerator_Ptr m_randomNumberGenerator;

  /** The total number of examples that have been added to the reservoir over time. */
  size_t m_seenExamples;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a reservoir that can store at most the specified number of examples of each class.
   *
   * Adding more than the specified number of examples of a particular class to the reservoir may result
   * in some of the older examples for that class being (randomly) discarded.
   *
   * \param maxClassSize          The maximum number of examples of each class allowed in the reservoir at any one time.
   * \param randomNumberGenerator A random number generator.
   */
  ExampleReservoir(size_t maxClassSize, const tvgutil::RandomNumberGenerator_Ptr& randomNumberGenerator)
  : m_curSize(0), m_histogram(new Histogram<Label>), m_maxClassSize(maxClassSize), m_randomNumberGenerator(randomNumberGenerator), m_seenExamples(0)
  {}

  ExampleReservoir(size_t maxClassSize, const tvgutil::RandomNumberGenerator_Ptr& randomNumberGenerator, size_t curSize, std::map<Label,std::vector<boost::shared_ptr<Example<int> > > > examples, const Histogram_Ptr& histogram, size_t seenExamples) 
  : m_curSize(curSize), m_histogram(histogram), m_maxClassSize(maxClassSize), m_randomNumberGenerator(randomNumberGenerator), m_seenExamples(seenExamples)
  {}
  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Adds an example to the reservoir.
   *
   * If there is already a full complement of examples for the class corresponding to the new example's label in the reservoir,
   * an older example of that class may be (randomly) discarded to make space for the new one. If not, the new example itself
   * is discarded.
   *
   * \param example The example to be added.
   * \return        true, if the example was actually added to the reservoir, or false otherwise.
   */
  bool add_example(const Example_CPtr& example)
  {
    bool changed = false;

    std::vector<Example_CPtr>& examplesForClass = m_examples[example->get_label()];
    if(examplesForClass.size() < m_maxClassSize)
    {
      // If we haven't yet reached the maximum number of examples for this class, simply add the new one.
      examplesForClass.push_back(example);
      ++m_curSize;
      changed = true;
    }
    else
    {
      // Otherwise, randomly decide whether or not to replace one of the existing examples for this class with the new one.
      size_t binSize = m_histogram->get_bins().find(example->get_label())->second;
      size_t k = m_randomNumberGenerator->generate_int_from_uniform(0, static_cast<int>(binSize) - 1);
      if(k < examplesForClass.size())
      {
        examplesForClass[k] = example;
        changed = true;
      }
    }

    m_histogram->add(example->get_label());
    ++m_seenExamples;
    return changed;
  }

  /**
   * \brief Clears the reservoir.
   */
  void clear()
  {
    m_examples.clear();
    m_histogram.reset();
    m_randomNumberGenerator.reset();
  }

  /**
   * \brief Gets the total number of examples currently in the reservoir.
   *
   * \return The total number of examples currently in the reservoir.
   */
  size_t current_size() const
  {
    return m_curSize;
  }

  /**
   * \brief Gets the per-class ratios between the total number of examples seen for a class and the number of examples currently in the reservoir.
   *
   * \return  The per-class ratios between the total number of examples seen for a class and the number of examples currently in the reservoir.
   */
  std::map<Label,float> get_class_multipliers() const
  {
    std::map<Label,float> result;

    const std::map<Label,size_t>& bins = m_histogram->get_bins();
    typename std::map<Label,std::vector<Example_CPtr> >::const_iterator it = m_examples.begin(), iend = m_examples.end();
    typename std::map<Label,size_t>::const_iterator jt = bins.begin();
    for(; it != iend; ++it, ++jt)
    {
      assert(it->first == jt->first);
      result.insert(std::make_pair(it->first, static_cast<float>(jt->second) / it->second.size()));
    }

    return result;
  }

  /**
   * \brief Gets the examples currently in the reservoir.
   *
   * \return  The examples currently in the reservoir.
   */
  std::vector<Example_CPtr> get_examples() const
  {
    std::vector<Example_CPtr> examples;
    examples.reserve(m_curSize);
    for(typename std::map<Label,std::vector<Example_CPtr> >::const_iterator it = m_examples.begin(), iend = m_examples.end(); it != iend; ++it)
    {
      std::copy(it->second.begin(), it->second.end(), std::back_inserter(examples));
    }
    return examples;
  }

  /**
   * \brief Gets the histogram of the label distribution of all of the examples that have ever been added to the reservoir.
   *
   * \return  The histogram of the label distribution of all of the examples that have ever been added to the reservoir.
   */
  Histogram_CPtr get_histogram() const
  {
    return m_histogram;
  }

  /**
   * \brief Gets the total number of examples that have been added to the reservoir over time.
   *
   * \return The total number of examples that have been added to the reservoir over time.
   */
  size_t seen_examples() const
  {
    return m_seenExamples;
  }

  //#################### STREAM OPERATORS ####################

  /**
   * \brief Outputs a reservoir to a stream.
   *
   * \param os  The stream to which to output the reservoir.
   * \param rhs The reservoir to output.
   * \return    The stream.
   */
  friend std::ostream& operator<<(std::ostream& os, const ExampleReservoir& rhs)
  {
    for(typename std::vector<Example_CPtr>::const_iterator it = rhs.m_examples.begin(), iend = rhs.m_examples.end(); it != iend; ++it)
    {
      os << (*it)->get_label() << ' ';
    }

    return os;
  }

  //#################### SERIALIZATION #################### 
private:
  friend class boost::serialization::access;
  template<typename Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    /*ar & m_curSize;
    ar & m_examples;
    ar & m_histogram;
    ar & m_maxClassSize;
    ar & m_randomNumberGenerator;
    ar & m_seenExamples;*/
  }

  template<class Archive>
  friend void boost::serialization::save_construct_data(Archive& ar, const rafl::ExampleReservoir<int> *exampleReservoir, const unsigned int file_version);

  template<class Archive>
  friend void boost::serialization::load_construct_data(Archive& ar, rafl::ExampleReservoir<int> *exampleReservoir, const unsigned int file_version);
};

}

namespace boost { namespace serialization {
template<class Archive>
inline void save_construct_data(Archive& ar, const rafl::ExampleReservoir<int> *exampleReservoir, const unsigned int file_version)
{
  ar << exampleReservoir->m_curSize;
  ar << exampleReservoir->m_examples;
  ar << exampleReservoir->m_histogram;
  ar << exampleReservoir->m_maxClassSize;
  ar << exampleReservoir->m_randomNumberGenerator;
  ar << exampleReservoir->m_seenExamples;
}

template<class Archive>
inline void load_construct_data(Archive& ar, rafl::ExampleReservoir<int> *exampleReservoir, const unsigned int file_version)
{
  //Retrieve data from archive required to construct new instance.
  size_t curSize;
  ar >> curSize;

  std::map<int, std::vector<boost::shared_ptr<rafl::Example<int> > > >examples;
  ar >> examples;

  boost::shared_ptr<rafl::Histogram<int> > histogram;
  ar >> histogram;

  size_t maxClassSize;
  ar >> maxClassSize;

  tvgutil::RandomNumberGenerator_Ptr randomNumberGenerator;
  ar >> randomNumberGenerator;

  size_t seenExamples;
  ar >> seenExamples;

  ::new(exampleReservoir)rafl::ExampleReservoir<int>(maxClassSize, randomNumberGenerator, curSize, examples, histogram, seenExamples);
}
}}

#endif
