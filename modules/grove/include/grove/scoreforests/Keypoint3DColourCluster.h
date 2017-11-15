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

/**
 * \brief This struct represents a 3D modal cluster of points with an associated colour.
 *        Used during the camera pose regression.
 */
struct Keypoint3DColourCluster
{
  /** The colour associated to the cluster. */
  Vector3u colour;

  /** The determinant of the covariance matrix. */
  float determinant;

  /** The number of points part of the modal cluster. */
  int nbInliers;

  /** The position (in world coordinates) of the cluster. */
  Vector3f position;

  /** The inverse covariance matrix of the points belonging to the cluster. Needed to compute Mahalanobis distances. */
  Matrix3f positionInvCovariance;
};

typedef ORUtils::MemoryBlock<Keypoint3DColourCluster> Keypoint3DColourClusterMemoryBlock;
typedef boost::shared_ptr<Keypoint3DColourClusterMemoryBlock> Keypoint3DColourClusterMemoryBlock_Ptr;

}

#endif
