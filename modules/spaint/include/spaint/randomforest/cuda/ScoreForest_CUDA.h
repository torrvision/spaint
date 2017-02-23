/**
 * spaint: ScoreForest_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SCOREFORESTCUDA
#define H_SPAINT_SCOREFORESTCUDA

#include "../interface/ScoreForest.h"

namespace spaint
{
class ScoreForest_CUDA: public ScoreForest
{
public:
  explicit ScoreForest_CUDA(const std::string &fileName);

  virtual Prediction3DColour get_prediction(size_t treeIdx, size_t leafIdx) const;

protected:
  virtual void find_leaves(const RGBDPatchDescriptorImage_CPtr &descriptors,
      LeafIndicesImage_Ptr &leaf_indices) const;
  virtual void get_predictions(const LeafIndicesImage_Ptr &leaf_indices,
      ScorePredictionsImage_Ptr &predictions) const;

  //#################### SCOREFOREST INTEROP FUNCTIONS ####################
#ifdef WITH_SCOREFORESTS

public:
  explicit ScoreForest_CUDA(const EnsembleLearner &pretrained_forest);

#endif
};

}

#endif
