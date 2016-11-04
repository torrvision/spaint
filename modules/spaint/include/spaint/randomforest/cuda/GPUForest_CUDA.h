/**
 * spaint: GPUForest_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_GPUFORESTCUDA
#define H_SPAINT_GPUFORESTCUDA

#include "../interface/GPUForest.h"

namespace spaint
{
class GPUForest_CUDA: public GPUForest
{
public:
  explicit GPUForest_CUDA(const std::string &fileName);

protected:
  void find_leaves(const RGBDPatchFeatureImage_CPtr &features,
      LeafIndicesImage_Ptr &leaf_indices) const;
  void get_predictions(const LeafIndicesImage_Ptr &leaf_indices,
      GPUForestPredictionsImage_Ptr &predictions) const;

  //#################### SCOREFOREST INTEROP FUNCTIONS ####################
#ifdef WITH_SCOREFORESTS

public:
  explicit GPUForest_CUDA(const EnsembleLearner &pretrained_forest);

#endif
};

}

#endif
