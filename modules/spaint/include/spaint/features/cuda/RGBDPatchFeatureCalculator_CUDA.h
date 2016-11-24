/**
 * spaint: RGBDPatchFeatureCalculator_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_RGBDPATCHFEATURECALCULATOR_CUDA
#define H_SPAINT_RGBDPATCHFEATURECALCULATOR_CUDA

#include "../interface/RGBDPatchFeatureCalculator.h"

namespace spaint
{
/**
 * \brief An instance of this class allows to compute features based on
 *        depth and colour differences in RGBD images using CUDA
 *
 *        The features are computed as described in:
 *        "Exploiting uncertainty in regression forests for accurate camera relocalization"
 *        by Valentin et al.
 */
class RGBDPatchFeatureCalculator_CUDA: public RGBDPatchFeatureCalculator
{
public:
  //#################### CONSTRUCTORS ####################
  /**
   * \brief Constructs an instance of the RGBDPatchFeatureCalculator_CUDA.
   */
  RGBDPatchFeatureCalculator_CUDA();

public:
  //#################### PUBLIC MEMBER FUNCTIONS ####################
  /** Override. */
  virtual void compute_feature(const ITMUChar4Image *rgbImage,
      const ITMFloatImage *depthImage, const Vector4f &intrinsics,
      Keypoint3DColourImage *keypointsImage,
      RGBDPatchDescriptorImage *featuresImage,
      const Matrix4f &cameraPose) const;
};
}

#endif
