/**
 * rafl: ExampleReservoir.h
 */

#ifndef H_RAFL_EXAMPLERESERVOIR
#define H_RAFL_EXAMPLERESERVOIR

#include <iosfwd>
#include <vector>

#include <tvgutil/SharedPtr.h>

namespace rafl {

/**
 * \brief An instance of an instantiation of this class template represents a reservoir to store the examples for a node.
 */
class ExampleReservoir
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The IDs of the examples in the reservoir. */
  std::vector<int> m_exampleIDs;

  /** The maximum number of examples allowed in the reservoir at any one time. */
  size_t m_maxSize;

  /** The number of examples that have been added to the reservoir over time. */
  size_t m_seenExamples;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Constructs a reservoir that can store at most the specified number of examples.
   *
   * Adding more than the specified number of examples to the reservoir may result in some
   * of the older examples being (randomly) discarded.
   *
   * \param maxSize The maximum number of examples allowed in the reservoir at any one time.
   * \param seed    The seed to use for the reservoir's random number generator.
   */
  explicit ExampleReservoir(size_t maxSize, unsigned int seed);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Adds an example to the reservoir.
   *
   * If the reservoir is currently full, an older example may be (randomly) discarded to
   * make space for the new example.
   *
   * \param exampleID The ID of the example to be added.
   */
  void add_example(int exampleID);

  //#################### FRIENDS ####################

  friend std::ostream& operator<<(std::ostream& os, const ExampleReservoir& rhs);
};

//#################### STREAM OPERATORS ####################

std::ostream& operator<<(std::ostream& os, const ExampleReservoir& rhs);

}

#endif
