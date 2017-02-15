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
 * \brief An instance of this class can be used to compute features based on depth and colour
 *        differences in RGBD images using CUDA.
 *
 * The features are computed as described by Valentin et al. in "Exploiting Uncertainty in
 * Regression Forests for Accurate Camera Relocalization".
 */
class RGBDPatchFeatureCalculator_CUDA : public RGBDPatchFeatureCalculator
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a CUDA-based RGBD patch feature calculator.
   */
  RGBDPatchFeatureCalculator_CUDA();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void compute_feature(const ITMUChar4Image *rgbImage, const ITMFloatImage *depthImage, const Vector4f &intrinsics,
                               Keypoint3DColourImage *keypointsImage, RGBDPatchDescriptorImage *featuresImage, const Matrix4f &cameraPose) const;
};

}

#endif
