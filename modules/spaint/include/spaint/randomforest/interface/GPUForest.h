/**
 * spaint: GPUForest.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_GPUFOREST
#define H_SPAINT_GPUFOREST

#include "../../util/ITMImagePtrTypes.h"

#include <Learner.hpp>

#include "../../features/interface/RGBDPatchFeatureCalculator.h"

namespace spaint
{
struct GPUForestNode
{
  int leftChildIdx; // No need to store the right child, it's left + 1
  int leafIdx;      // Index of the associated leaf (-1 if the node is not a leaf)
  int featureIdx;   // Index of the feature to evaluate;
  float featureThreshold; // Feature threshold
};

typedef ORUtils::Image<GPUForestNode> GPUForestImage;
typedef boost::shared_ptr<ORUtils::Image<GPUForestNode> > GPUForestImage_Ptr;
typedef boost::shared_ptr<const ORUtils::Image<GPUForestNode> > GPUForestImage_CPtr;

class GPUForest
{
public:
  explicit GPUForest(const EnsembleLearner &pretrained_forest);
  virtual ~GPUForest();

  virtual void evaluate_forest(const RGBDPatchFeatureImage_CPtr &features, ITMIntImage_Ptr &leaf_indices) const = 0;

protected:
  GPUForestImage_Ptr m_forestImage;

private:
  int convert_node(const Learner *learner, int node_idx, int tree_idx, int n_trees, int output_idx, int first_free_idx, GPUForestNode *gpu_nodes);
};

typedef boost::shared_ptr<GPUForest> GPUForest_Ptr;
typedef boost::shared_ptr<const GPUForest> GPUForest_CPtr;

}

#endif
