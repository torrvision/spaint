/**
 * grove: Prediction3DColour.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_PREDICTION3DCOLOUR
#define H_GROVE_PREDICTION3DCOLOUR

#include <ORUtils/PlatformIndependence.h>
#include <ORUtils/Vector.h>

#include <itmx/ITMImagePtrTypes.h>

#include "Mode3DColour.h"

namespace grove {

/**
 * \brief An instance of this struct represents a multi-modal prediction of 3D point clusters. Used during the camera pose regression.
 */
struct Prediction3DColour
{
  /** The maximum number of modal clusters associated to the prediction. */
  enum { MAX_MODES = 10 };

  /** The modal clusters. */
  Mode3DColour modes[MAX_MODES];

  /** The actual number of modes stored in the struct. */
  int nbModes;

  /**
   * \brief Given a 3D point, find the closest 3D Mode according to the Mahalanobis distance.
   *
   * \param v        A 3D point in world coordinates.
   * \param maxScore Will contain the energy associated to the closest mode.
   *
   * \return The index of the closest modal cluster. -1 if nbModes is 0.
   *
   * TODO: This should become a free function and be in a different file.
   */
  _CPU_AND_GPU_CODE_
  int get_best_mode_and_energy(const Vector3f &v, float &maxScore) const
  {
    const float exponent = powf(2.0f * static_cast<float>(M_PI), 3);

    int argmax = -1;
    maxScore = -1.f; // If set to 0 the comparison later fails for very small values
//    maxScore = 0.0f;

    // Iterate over all the modal clusters stored in the struct.
    for (int m = 0; m < nbModes; ++m)
    {
      const Vector3f diff = v - modes[m].position;

      // This is the textbook implementation of Mahalanobis distance
      // Helpers::MahalanobisSquared3x3 used in the scoreforests code seems wrong.
      const float mahalanobisSq = dot(diff,
          modes[m].positionInvCovariance * diff);
      const float descriptiveStatistics = expf(-0.5f * mahalanobisSq);

      const float normalization = 1.0f / sqrtf(modes[m].determinant * exponent);
      const float evalGaussian = normalization * descriptiveStatistics;

      const float nbPts = static_cast<float>(modes[m].nbInliers);
      const float score = nbPts * evalGaussian;

      // If the point is "closer" to the center of the anisotropic gaussian associated to this mode, store its index.
      if (score > maxScore)
      {
        maxScore = score;
        argmax = m;
      }
    }

    return argmax;
  }

  /**
   * \brief Given a 3D point, find the closest 3D Mode according to the Mahalanobis distance.
   *
   * \param v        A 3D point in world coordinates.
   *
   * \return The index of the closest modal cluster. -1 if nbModes is 0.
   */
  _CPU_AND_GPU_CODE_
  int get_best_mode(const Vector3f &v) const
  {
    float energy;
    return get_best_mode_and_energy(v, energy);
  }
};

//#################### TYPEDEFS ####################

typedef ORUtils::MemoryBlock<Prediction3DColour> ScorePredictionsBlock;
typedef boost::shared_ptr<ScorePredictionsBlock> ScorePredictionsBlock_Ptr;
typedef boost::shared_ptr<const ScorePredictionsBlock> ScorePredictionsBlock_CPtr;

typedef ORUtils::Image<Prediction3DColour> ScorePredictionsImage;
typedef boost::shared_ptr<ScorePredictionsImage> ScorePredictionsImage_Ptr;
typedef boost::shared_ptr<const ScorePredictionsImage> ScorePredictionsImage_CPtr;

}

#endif
