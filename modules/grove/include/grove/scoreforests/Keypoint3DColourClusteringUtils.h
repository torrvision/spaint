/**
 * grove: Keypoint3DColourClusteringUtils.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_KEYPOINT3DCOLOURCLUSTERINGUTILS
#define H_GROVE_KEYPOINT3DCOLOURCLUSTERINGUTILS

#include <ORUtils/PlatformIndependence.h>

#include "../keypoints/Keypoint3DColour.h"
#include "Mode3DColour.h"

namespace grove {

/**
 * \brief Computes the squared distance between two Keypoint3DColour instances.
 *
 * \param a A Keypoint3DColour.
 * \param b Another Keypoint3DColour.
 *
 * \return  The squared distance between a and b.
 */
_CPU_AND_GPU_CODE_
inline float distance_squared(const Keypoint3DColour& a, const Keypoint3DColour& b)
{
  const Vector3f diff = b.position - a.position;
  return dot(diff, diff);
}

/**
 * \brief Constructs a modal cluster given a certain number of example keypoints.
 *
 * \note  Each example has an associated key, only examples with key equal to "key"
 *        are used to construct the cluster.
 *
 * \param key           Only examples having key equal to this parameter are used to compute the cluster parameters.
 * \param examples      The example keypoints that potentially may be part of the cluster.
 * \param exampleKeys   Keys associated to each example.
 * \param examplesCount The number of examples.
 * \param outputCluster The constructed cluster.
 */
_CPU_AND_GPU_CODE_
inline void create_cluster_from_examples(int key, const Keypoint3DColour *examples, const int *exampleKeys, int examplesCount, Mode3DColour& outputCluster)
{
  // Compute position and colour mean.
  int sampleCount = 0;
  Vector3f positionMean(0.f);
  Vector3f colourMean(0.f);

  // Iterate over all examples and use only those belonging to cluster key.
  for (int sampleIdx = 0; sampleIdx < examplesCount; ++sampleIdx)
  {
    const int sampleCluster = exampleKeys[sampleIdx];
    if (sampleCluster == key)
    {
      const Keypoint3DColour &sample = examples[sampleIdx];

      ++sampleCount;
      positionMean += sample.position;
      colourMean += sample.colour.toFloat();
    }
  }

  // This mode is invalid.
  if (sampleCount <= 1)
  {
// Should never reach this point since we should have checked minClusterSize earlier.
#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
    printf("create_cluster_from_examples: got a cluster with less than 2 elements.\n");
    asm("trap;");
#else
    throw std::runtime_error("create_cluster_from_examples: got a cluster with less than 2 elements.");
#endif
  }

  positionMean /= static_cast<float>(sampleCount);
  colourMean /= static_cast<float>(sampleCount);

  // Now iterate again and compute the covariance.
  Matrix3f positionCovariance;
  positionCovariance.setZeros();

  for (int sampleIdx = 0; sampleIdx < examplesCount; ++sampleIdx)
  {
    const int sampleCluster = exampleKeys[sampleIdx];

    if (sampleCluster == key)
    {
      const Keypoint3DColour &sample = examples[sampleIdx];

      for (int i = 0; i < 3; ++i)
      {
        for (int j = 0; j < 3; ++j)
        {
          positionCovariance.m[i * 3 + j] +=
              (sample.position.v[i] - positionMean.v[i]) * (sample.position.v[j] - positionMean.v[j]);
        }
      }
    }
  }

  positionCovariance /= static_cast<float>(sampleCount - 1);
  const float positionDeterminant = positionCovariance.det();

  // Fill the cluster.
  outputCluster.colour = colourMean.toUChar();
  outputCluster.determinant = positionDeterminant;
  outputCluster.nbInliers = sampleCount;
  outputCluster.position = positionMean;
  positionCovariance.inv(outputCluster.positionInvCovariance);
}

}

#endif
