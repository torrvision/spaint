/**
 * spaint: SCoReForest_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SCOREFORESTCUDA
#define H_SPAINT_SCOREFORESTCUDA

#include "../interface/SCoReForest.h"

namespace spaint
{
class SCoReForest_CUDA: public SCoReForest
{
public:
  explicit SCoReForest_CUDA(const std::string &fileName);

  virtual GPUForestPrediction get_prediction(size_t treeIdx,
      size_t leafIdx) const;

protected:
  virtual void find_leaves(const RGBDPatchFeatureImage_CPtr &features,
      LeafIndicesImage_Ptr &leaf_indices) const;
  virtual void get_predictions(const LeafIndicesImage_Ptr &leaf_indices,
      GPUForestPredictionsImage_Ptr &predictions) const;

  //#################### SCOREFOREST INTEROP FUNCTIONS ####################
#ifdef WITH_SCOREFORESTS

public:
  explicit SCoReForest_CUDA(const EnsembleLearner &pretrained_forest);

#endif
};

}

#endif
