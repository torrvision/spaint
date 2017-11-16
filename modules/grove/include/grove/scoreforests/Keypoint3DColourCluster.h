/**
 * grove: Keypoint3DColourCluster.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_KEYPOINT3DCOLOURCLUSTER
#define H_GROVE_KEYPOINT3DCOLOURCLUSTER

#include <boost/shared_ptr.hpp>

#include <ITMLib/Utils/ITMMath.h>

#include <ORUtils/MemoryBlock.h>

namespace grove {

//#################### MAIN TYPE ####################

/**
 * \brief An instance of this struct represents a modal cluster of 3D points with associated colours, as used during camera pose regression.
 */
struct Keypoint3DColourCluster
{
  //#################### PUBLIC VARIABLES ####################

  /** The colour associated to the cluster. */
  Vector3u colour;

  /** The determinant of the covariance matrix. */
  float determinant;

  /** The number of points that belong to the cluster. */
  int nbInliers;

  /** The position (in world coordinates) of the cluster. */
  Vector3f position;

  /** The inverse covariance matrix of the points belonging to the cluster. This is needed to compute Mahalanobis distances. */
  Matrix3f positionInvCovariance;
};

//#################### TYPEDEFS ####################

typedef ORUtils::MemoryBlock<Keypoint3DColourCluster> Keypoint3DColourClusterMemoryBlock;
typedef boost::shared_ptr<Keypoint3DColourClusterMemoryBlock> Keypoint3DColourClusterMemoryBlock_Ptr;

//#################### FUNCTIONS ####################

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
inline void create_cluster_from_examples(int key, const Keypoint3DColour *examples, const int *exampleKeys, int examplesCount, Keypoint3DColourCluster& outputCluster)
{
  // Compute position and colour mean.
  int sampleCount = 0;
  Vector3f positionMean(0.f);
  Vector3f colourMean(0.f);

  // Iterate over all examples and use only those belonging to cluster key.
  for(int sampleIdx = 0; sampleIdx < examplesCount; ++sampleIdx)
  {
    const int sampleCluster = exampleKeys[sampleIdx];
    if(sampleCluster == key)
    {
      const Keypoint3DColour &sample = examples[sampleIdx];

      ++sampleCount;
      positionMean += sample.position;
      colourMean += sample.colour.toFloat();
    }
  }

  // This mode is invalid.
  if(sampleCount <= 1)
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

  for(int sampleIdx = 0; sampleIdx < examplesCount; ++sampleIdx)
  {
    const int sampleCluster = exampleKeys[sampleIdx];

    if(sampleCluster == key)
    {
      const Keypoint3DColour& sample = examples[sampleIdx];

      for(int i = 0; i < 3; ++i)
      {
        for(int j = 0; j < 3; ++j)
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

/**
 * \brief Computes the squared distance between two 3D colour keypoints.
 *
 * \param a The first 3D colour keypoint.
 * \param b The second 3D colour keypoint.
 * \return  The squared distance between a and b.
 */
_CPU_AND_GPU_CODE_
inline float distance_squared(const Keypoint3DColour& a, const Keypoint3DColour& b)
{
  const Vector3f diff = b.position - a.position;
  return dot(diff, diff);
}

}

#endif
