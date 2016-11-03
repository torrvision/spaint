/**
 * spaint: GPUForestTypes.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_GPUFORESTTYPES
#define H_SPAINT_GPUFORESTTYPES

#include "../../util/ITMImagePtrTypes.h"
#include "ORUtils/Vector.h"
#include "ORUtils/PlatformIndependence.h"

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
  enum
  {
    MAX_MODES = 10
  };

  GPUForestMode modes[MAX_MODES];
  int nbModes;

  _CPU_AND_GPU_CODE_
  int get_best_mode_and_energy(const Vector3f &v, float &maxScore) const
  {
    const float exponent = powf(2.0f * M_PI, 3);

    int argmax = -1;
    maxScore = -1.f; // If set to 0 the comparison later fails for very small values
//    maxScore = 0.0f; // If set to 0 the comparison later fails for very small values
//    maxScore = std::numeric_limits<float>::lowest();

    for (int m = 0; m < nbModes; ++m)
    {
      const float nbPts = static_cast<float>(modes[m].nbInliers);
      const Vector3f diff = v - modes[m].position;

      const float normalization = 1.0 / sqrtf(modes[m].determinant * exponent);
      // This is the textbook implementation of Mahalanobis distance
      // Helpers::MahalanobisSquared3x3 used in the original code seems wrong
      const float mahalanobisSq = dot(diff,
          modes[m].positionInvCovariance * diff);
      const float descriptiveStatistics = expf(-0.5f * mahalanobisSq);
      const float evalGaussian = normalization * descriptiveStatistics;
      const float score = nbPts * evalGaussian;

      if (score > maxScore)
      {
        maxScore = score;
        argmax = m;
      }
    }

    return argmax;
  }

  _CPU_AND_GPU_CODE_
  int get_best_mode(const Vector3f &v) const
  {
    float energy;
    return get_best_mode_and_energy(v, energy);
  }
};

typedef ORUtils::MemoryBlock<GPUForestPrediction> GPUForestPredictionsBlock;
typedef boost::shared_ptr<GPUForestPredictionsBlock> GPUForestPredictionsBlock_Ptr;
typedef boost::shared_ptr<const GPUForestPredictionsBlock> GPUForestPredictionsBlock_CPtr;

typedef ORUtils::Image<GPUForestPrediction> GPUForestPredictionsImage;
typedef boost::shared_ptr<GPUForestPredictionsImage> GPUForestPredictionsImage_Ptr;
typedef boost::shared_ptr<const GPUForestPredictionsImage> GPUForestPredictionsImage_CPtr;

enum
{
  GPUFOREST_NTREES = 5 // Max number of trees
};

typedef ORUtils::VectorX<int, GPUFOREST_NTREES> LeafIndices;
typedef ORUtils::Image<LeafIndices> LeafIndicesImage;
typedef boost::shared_ptr<LeafIndicesImage> LeafIndicesImage_Ptr;
typedef boost::shared_ptr<const LeafIndicesImage> LeafIndicesImage_CPtr;
}

#endif
