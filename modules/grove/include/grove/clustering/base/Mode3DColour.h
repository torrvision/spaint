/**
 * grove: Mode3DColour.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_MODE3DCOLOUR
#define H_GROVE_MODE3DCOLOUR

#include <ORUtils/Vector.h>

namespace grove {

/**
 * \brief This struct represents a 3D modal cluster of points with an associated colour, used during the camera pose regression.
 */
struct Mode3DColour
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

}

#endif
