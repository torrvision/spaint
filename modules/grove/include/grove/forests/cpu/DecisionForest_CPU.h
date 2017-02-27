/**
 * grove: DecisionForest_CPU.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_DECISIONFORESTCPU
#define H_GROVE_DECISIONFORESTCPU

#include "../interface/DecisionForest.h"

namespace grove {

template <typename DescriptorType, int TreeCount>
class DecisionForest_CPU : public DecisionForest<DescriptorType, TreeCount>
{
public:
  using DecisionForest<DescriptorType, TreeCount>::TREE_COUNT;

  using typename DecisionForest<DescriptorType, TreeCount>::DescriptorImage;
  using typename DecisionForest<DescriptorType, TreeCount>::DescriptorImage_Ptr;
  using typename DecisionForest<DescriptorType, TreeCount>::DescriptorImage_CPtr;

  using typename DecisionForest<DescriptorType, TreeCount>::LeafIndices;
  using typename DecisionForest<DescriptorType, TreeCount>::LeafIndicesImage;
  using typename DecisionForest<DescriptorType, TreeCount>::LeafIndicesImage_Ptr;
  using typename DecisionForest<DescriptorType, TreeCount>::LeafIndicesImage_CPtr;

  using typename DecisionForest<DescriptorType, TreeCount>::NodeEntry;
  using typename DecisionForest<DescriptorType, TreeCount>::NodeImage;
  using typename DecisionForest<DescriptorType, TreeCount>::NodeImage_Ptr;
  using typename DecisionForest<DescriptorType, TreeCount>::NodeImage_CPtr;

public:
  DecisionForest_CPU(const std::string& fileName);

  virtual void find_leaves(const DescriptorImage_CPtr& descriptors, LeafIndicesImage_Ptr& leafIndices) const;

#ifdef WITH_SCOREFORESTS
public:
  explicit DecisionForest_CPU(const EnsembleLearner &pretrainedForest);
#endif
};

}

#endif
