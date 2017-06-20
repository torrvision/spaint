/**
 * grove: DecisionForest_CPU.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_DECISIONFOREST_CPU
#define H_GROVE_DECISIONFOREST_CPU

#include "../interface/DecisionForest.h"

namespace grove {

/**
 * \brief An instance of this class represents a binary decision forest composed of a fixed number
 *        of trees. The evaluation is performed on the CPU.
 *
 * \note  Training is not performed by this class. We use the node indexing technique described in:
 *        "Toby Sharp, Implementing decision trees and forests on a GPU. (2008)"
 *
 * \param DescriptorType  The type of descriptor used to find the leaves. Must have a floating-point member array named
 *                        "data".
 * \param TreeCount       The number of trees in the forest. Fixed at compilation time to allow the definition of a data
 *                        type representing the leaf indices.
 */
template <typename DescriptorType, int TreeCount>
class DecisionForest_CPU : public DecisionForest<DescriptorType, TreeCount>
{
  //#################### ENUMS ####################
public:
  using DecisionForest<DescriptorType, TreeCount>::TREE_COUNT;

  //#################### TYPEDEFS ####################
public:
  using typename DecisionForest<DescriptorType, TreeCount>::DescriptorImage;
  using typename DecisionForest<DescriptorType, TreeCount>::DescriptorImage_Ptr;
  using typename DecisionForest<DescriptorType, TreeCount>::DescriptorImage_CPtr;

  using typename DecisionForest<DescriptorType, TreeCount>::LeafIndices;
  using typename DecisionForest<DescriptorType, TreeCount>::LeafIndicesImage;
  using typename DecisionForest<DescriptorType, TreeCount>::LeafIndicesImage_Ptr;
  using typename DecisionForest<DescriptorType, TreeCount>::LeafIndicesImage_CPtr;

  using typename DecisionForest<DescriptorType, TreeCount>::NodeEntry;

  //#################### CONSTRUCTORS ####################
public:
  explicit DecisionForest_CPU(const std::string &fileName);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  virtual void find_leaves(const DescriptorImage_CPtr &descriptors, LeafIndicesImage_Ptr &leafIndices) const;

//#################### SCOREFOREST INTEROP FUNCTIONS ####################
#ifdef WITH_SCOREFORESTS
  //#################### CONSTRUCTORS ####################
public:
  explicit DecisionForest_CPU(const EnsembleLearner &pretrainedForest);
#endif
};

} // namespace grove

#endif
