/**
 * spaint: GPUForest.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_GPUFOREST
#define H_SPAINT_GPUFOREST

#include "../../util/ITMImagePtrTypes.h"

#include <Learner.hpp>

#include "../../features/interface/RGBDPatchFeatureCalculator.h"
#include "GPUReservoir.h"
#include "GPUForestTypes.h"

#include "ORUtils/Vector.h"

namespace spaint
{

class GPUForest
{
  // Typedefs
public:
  static const int RESERVOIR_SIZE = 1000; // Max number samples in a leaf reservoir
  static const int NTREES = GPUFOREST_NTREES; // Max number of trees

  typedef spaint::LeafIndices LeafIndices; // TODO: remove
  typedef ORUtils::Image<LeafIndices> LeafIndicesImage;
  typedef boost::shared_ptr<LeafIndicesImage> LeafIndicesImage_Ptr;
  typedef boost::shared_ptr<const LeafIndicesImage> LeafIndicesImage_CPtr;

public:
  explicit GPUForest(const EnsembleLearner &pretrained_forest);
  virtual ~GPUForest();

  void reset_predictions();
  void evaluate_forest(const RGBDPatchFeatureImage_CPtr &features,
      GPUForestPredictionsImage_Ptr &predictions);
  void add_features_to_forest(const RGBDPatchFeatureImage_CPtr &features);

protected:
  GPUForestImage_Ptr m_forestImage;
  GPUForestPredictionsBlock_Ptr m_predictionsBlock;
  std::vector<PredictionGaussianMean> m_leafPredictions;
  PositionReservoir_Ptr m_leafReservoirs;

  virtual void find_leaves(const RGBDPatchFeatureImage_CPtr &features,
      LeafIndicesImage_Ptr &leaf_indices) const = 0;
  virtual void get_predictions(const LeafIndicesImage_Ptr &leaf_indices,
      GPUForestPredictionsImage_Ptr &predictions) const = 0;

private:
  int convert_node(const Learner *learner, int node_idx, int tree_idx,
      int n_trees, int output_idx, int first_free_idx,
      GPUForestNode *gpu_nodes);
  void convert_predictions();

  LeafIndicesImage_Ptr m_leafImage;

  boost::shared_ptr<MeanShift3D> m_ms3D;
};

typedef boost::shared_ptr<GPUForest> GPUForest_Ptr;
typedef boost::shared_ptr<const GPUForest> GPUForest_CPtr;

}

#endif
