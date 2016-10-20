/**
 * spaint: GPUForest.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_GPUFOREST
#define H_SPAINT_GPUFOREST

#include "../../util/ITMImagePtrTypes.h"

#include <Learner.hpp>

#include "../../features/interface/RGBDPatchFeatureCalculator.h"

#include "ORUtils/Vector.h"

namespace spaint
{
struct GPUForestNode
{
  int leftChildIdx; // No need to store the right child, it's left + 1
  int leafIdx;    // Index of the associated leaf (-1 if the node is not a leaf)
  int featureIdx;   // Index of the feature to evaluate;
  float featureThreshold; // Feature threshold
};

typedef ORUtils::Image<GPUForestNode> GPUForestImage;
typedef boost::shared_ptr<ORUtils::Image<GPUForestNode> > GPUForestImage_Ptr;
typedef boost::shared_ptr<const ORUtils::Image<GPUForestNode> > GPUForestImage_CPtr;

struct GPUForestMode
{
  Vector3f position;
//  Matrix3f positionCovariance; // Seems not needed
  Matrix3f positionInvCovariance; // Needed to compute Mahalanobis distance
  float determinant;

  Vector3u colour;

  int nbInliers;
};

struct GPUForestPrediction
{
  static const int MAX_MODES = 10;

  GPUForestMode modes[MAX_MODES];
  int nbModes;

  int get_best_mode(const Vector3f &x) const;
  int get_best_mode_and_energy(const Vector3f &x, float& energy) const;
};

typedef ORUtils::MemoryBlock<GPUForestPrediction> GPUForestPredictionsBlock;
typedef boost::shared_ptr<GPUForestPredictionsBlock> GPUForestPredictionsBlock_Ptr;
typedef boost::shared_ptr<const GPUForestPredictionsBlock> GPUForestPredictionsBlock_CPtr;

class GPUForest
{
  // Typedefs
public:
  static const int NTREES = 5; // Max number of trees
  typedef ORUtils::VectorX<int, NTREES> LeafIndices;
  typedef ORUtils::Image<LeafIndices> LeafIndicesImage;
  typedef boost::shared_ptr<LeafIndicesImage> LeafIndicesImage_Ptr;
  typedef boost::shared_ptr<const LeafIndicesImage> LeafIndicesImage_CPtr;

public:
  explicit GPUForest(const EnsembleLearner &pretrained_forest);
  virtual ~GPUForest();

  virtual void evaluate_forest(const RGBDPatchFeatureImage_CPtr &features,
      LeafIndicesImage_Ptr &leaf_indices) const = 0;
  boost::shared_ptr<EnsemblePredictionGaussianMean> get_prediction_for_leaves(
      const LeafIndices &leaves);

protected:
  GPUForestImage_Ptr m_forestImage;
  GPUForestPredictionsBlock_Ptr m_predictionsBlock;
  std::vector<PredictionGaussianMean> m_leafPredictions;

private:
  int convert_node(const Learner *learner, int node_idx, int tree_idx,
      int n_trees, int output_idx, int first_free_idx,
      GPUForestNode *gpu_nodes);
  void convert_predictions();
};

typedef boost::shared_ptr<GPUForest> GPUForest_Ptr;
typedef boost::shared_ptr<const GPUForest> GPUForest_CPtr;

}

#endif
