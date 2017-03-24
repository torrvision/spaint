/**
 * grove: Keypoint3DColourClusteringUtils.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_KEYPOINT3DCOLOURCLUSTERINGUTILS
#define H_GROVE_KEYPOINT3DCOLOURCLUSTERINGUTILS

#include <ORUtils/PlatformIndependence.h>

#include "../../keypoints/Keypoint3DColour.h"
#include "Mode3DColour.h"

namespace grove {

_CPU_AND_GPU_CODE_
inline float distanceSquared(const Keypoint3DColour &a, const Keypoint3DColour &b)
{
  const Vector3f diff = b.position - a.position;
  return dot(diff, diff);
}

_CPU_AND_GPU_CODE_
inline void computeMode(
    const Keypoint3DColour *examples, const int *exampleKeys, int examplesCount, int key, Mode3DColour &outputMode)
{
  // compute position and colour mean
  int sampleCount = 0;
  Vector3f positionMean(0.f);
  Vector3f colourMean(0.f);

  // Iterate over all examples and use only those belonging to selectedClusterId
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

  // this mode is invalid..
  if (sampleCount <= 1)
  {
// Should never reach this point since we check minClusterSize earlier
#ifdef __CUDACC__
    printf("computeMode: got a cluster with less than 2 elements.\n");
    asm("trap;");
#else
    throw std::runtime_error("computeMode: got a cluster with less than 2 elements.");
#endif
  }

  positionMean /= static_cast<float>(sampleCount);
  colourMean /= static_cast<float>(sampleCount);

  // Now iterate again and compute the covariance
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

  // Fill the mode
  outputMode.nbInliers = sampleCount;
  outputMode.position = positionMean;
  outputMode.determinant = positionDeterminant;
  positionCovariance.inv(outputMode.positionInvCovariance);
  outputMode.colour = colourMean.toUChar();
}

} // namespace grove

#endif
