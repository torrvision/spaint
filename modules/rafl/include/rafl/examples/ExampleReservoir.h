/**
 * rafl: ExampleReservoir.h
 */

#ifndef H_RAFL_EXAMPLERESERVOIR
#define H_RAFL_EXAMPLERESERVOIR

#include <iosfwd>
#include <vector>

#include <tvgutil/RandomNumberGenerator.h>

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

  //#################### PRIVATE VARIABLES ####################
private:
  /** The examples in the reservoir. */
  std::vector<Example_CPtr> m_examples;

  /** The maximum number of examples allowed in the reservoir at any one time. */
  size_t m_maxSize;

  /** A random number generator. */
  tvgutil::RandomNumberGenerator_Ptr m_rng;

  /** The number of examples that have been added to the reservoir over time. */
  size_t m_seenExamples;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a reservoir that can store at most the specified number of examples.
   *
   * Adding more than the specified number of examples to the reservoir may result in some
   * of the older examples being (randomly) discarded.
   *
   * \param maxSize The maximum number of examples allowed in the reservoir at any one time.
   * \param rng     A random number generator.
   */
  explicit ExampleReservoir(size_t maxSize, const tvgutil::RandomNumberGenerator_Ptr& rng)
  : m_maxSize(maxSize), m_rng(rng), m_seenExamples(0)
  {}

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Adds an example to the reservoir.
   *
   * If the reservoir is currently full, an older example may be (randomly) discarded to
   * make space for the new example.
   *
   * \param example The example to be added.
   * \return        true, if the example was actually added to the reservoir, or false otherwise.
   */
  bool add_example(const Example_CPtr& example)
  {
    bool changed = false;

    if(m_seenExamples < m_maxSize)
    {
      m_examples.push_back(example);
      changed = true;
    }
    else
    {
      size_t k = m_rng->generate_int_in_range(0, static_cast<int>(m_seenExamples) - 1);
      if(k < m_maxSize)
      {
        m_examples[k] = example;
        changed = true;
      }
    }

    ++m_seenExamples;
    return changed;
  }

  size_t current_size() const
  {
    return m_examples.size();
  }

  size_t max_size() const
  {
    return m_maxSize;
  }

  //#################### FRIENDS ####################

  friend std::ostream& operator<<(std::ostream& os, const ExampleReservoir& rhs)
  {
    for(typename std::vector<Example_CPtr>::const_iterator it = rhs.m_examples.begin(), iend = rhs.m_examples.end(); it != iend; ++it)
    {
      os << (*it)->get_label() << ' ';
    }

    return os;
  }
};

}

#endif
