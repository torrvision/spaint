/**
 * rafl: RandomForest.h
 */

#ifndef H_RAFL_RANDOMFOREST
#define H_RAFL_RANDOMFOREST

#include <tvgutil/IDAllocator.h>

#include "DecisionTree.h"

namespace rafl {
  
/**
 * \brief An instance of an instantiation of this class template represents a random forest.
 */
template <typename Label>
class RandomForest
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<DecisionTree<Label> > Tree_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The global example buffer. */
  boost::shared_ptr<std::map<int,Example<Label> > > m_exampleBuffer;

  /** The ID allocator that is used to generate example IDs. */
  tvgutil::IDAllocator m_exampleIDAllocator;

  /** The decision trees that collectively make up the random forest. */
  std::vector<Tree_Ptr> m_trees;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a random forest.
   *
   * \param rng A random number generator.
   */
  explicit RandomForest(const tvgutil::RandomNumberGenerator_Ptr& rng)
  {
    // TODO
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Adds new training examples to the forest.
   *
   * \param examples  The examples to be added.
   */
  void add_examples(const std::vector<Example<Label> >& examples)
  {
    // Generate IDs for the new examples and add them to the global example buffer.
    std::vector<int> exampleIDs;
    exampleIDs.reserve(examples.size());
    for(typename std::vector<Example<Label> >::const_iterator it = examples.begin(), iend = examples.end(); it != iend; ++it)
    {
      int id = m_exampleIDAllocator.allocate();
      exampleIDs.push_back(id);
      m_exampleBuffer.insert(std::make_pair(id, *it));
    }

    // Add the new examples to the different trees.
    for(typename std::vector<Tree_Ptr>::const_iterator it = m_trees.begin(), iend = m_trees.end(); it != iend; ++it)
    {
      (*it)->add_examples(exampleIDs);
    }
  }
};

}

#endif
