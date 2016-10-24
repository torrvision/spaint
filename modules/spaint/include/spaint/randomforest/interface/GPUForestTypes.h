/**
 * spaint: GPUForestTypes.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_GPUFORESTTYPES
#define H_SPAINT_GPUFORESTTYPES

#include "../../util/ITMImagePtrTypes.h"
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

typedef ORUtils::Image<GPUForestPrediction> GPUForestPredictionsImage;
typedef boost::shared_ptr<GPUForestPredictionsImage> GPUForestPredictionsImage_Ptr;
typedef boost::shared_ptr<const GPUForestPredictionsImage> GPUForestPredictionsImage_CPtr;

static const int GPUFOREST_NTREES = 5; // Max number of trees

typedef ORUtils::VectorX<int, GPUFOREST_NTREES> LeafIndices;
typedef ORUtils::Image<LeafIndices> LeafIndicesImage;
typedef boost::shared_ptr<LeafIndicesImage> LeafIndicesImage_Ptr;
typedef boost::shared_ptr<const LeafIndicesImage> LeafIndicesImage_CPtr;
}

#endif
