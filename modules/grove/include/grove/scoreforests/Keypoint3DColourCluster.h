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
 * \brief Constructs a modal cluster from those examples in an input list of examples that have the specified key.
 *
 * \param key           The key associated with the cluster. Only examples with this key are used to compute the cluster parameters.
 * \param examples      An input list of examples.
 * \param exampleKeys   The keys associated with the examples (one per example).
 * \param exampleCount  The number of examples in the input list.
 * \param outputCluster The constructed cluster.
 */
_CPU_AND_GPU_CODE_
inline void create_cluster_from_examples(int key, const Keypoint3DColour *examples, const int *exampleKeys, int exampleCount, Keypoint3DColourCluster& outputCluster)
{
  // First, compute the cluster's position and colour by averaging the positions and colours of the examples that have the specified key.
  int nbInliers = 0;
  Vector3f positionMean(0.0f, 0.0f, 0.0f);
  Vector3f colourMean(0.0f, 0.0f, 0.0f);

  for(int exampleIdx = 0; exampleIdx < exampleCount; ++exampleIdx)
  {
    if(exampleKeys[exampleIdx] == key)
    {
      const Keypoint3DColour& example = examples[exampleIdx];
      positionMean += example.position;
      colourMean += example.colour.toFloat();
      ++nbInliers;
    }
  }

  if(nbInliers > 1)
  {
    positionMean /= static_cast<float>(nbInliers);
    colourMean /= static_cast<float>(nbInliers);
  }
  else
  {
    // If the cluster contains fewer than two examples, something has gone wrong, so signal an error. This should never
    // happen in practice, since the clusterer is designed to only construct clusters whose size is >= minClusterSize.
#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
    printf("Error: create_cluster_from_examples: One of the clusters has less than 2 elements.\n");
    asm("trap;");
#else
    throw std::runtime_error("Error: create_cluster_from_examples: One of the clusters has less than 2 elements.");
#endif
  }

  // Next, iterate again and compute the covariance matrix and its determinant.
  Matrix3f positionCovariance;
  positionCovariance.setZeros();

  for(int exampleIdx = 0; exampleIdx < exampleCount; ++exampleIdx)
  {
    if(exampleKeys[exampleIdx] == key)
    {
      const Keypoint3DColour& example = examples[exampleIdx];

      for(int i = 0; i < 3; ++i)
      {
        for(int j = 0; j < 3; ++j)
        {
          positionCovariance.m[i * 3 + j] += (example.position.v[i] - positionMean.v[i]) * (example.position.v[j] - positionMean.v[j]);
        }
      }
    }
  }

  positionCovariance /= static_cast<float>(nbInliers - 1);
  const float positionDeterminant = positionCovariance.det();

  // Finally, fill in the output cluster using the computed values.
  outputCluster.colour = colourMean.toUChar();
  outputCluster.determinant = positionDeterminant;
  outputCluster.nbInliers = nbInliers;
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
