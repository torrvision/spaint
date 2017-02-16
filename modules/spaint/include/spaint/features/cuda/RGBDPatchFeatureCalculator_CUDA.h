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
 * \brief An instance of this class can be used to compute features based on
 *        depth and colour differences in RGBD images using CUDA.
 *
 * The features are computed as described by Valentin et al. in "Exploiting
 * Uncertainty in Regression Forests for Accurate Camera Relocalization".
 *
 * \param KeypointType    The type of keypoints computed by this class.
 * \param DescriptorType  he type of descriptors computed by this class.
 */
template<typename KeypointType, typename DescriptorType>
class RGBDPatchFeatureCalculator_CUDA : public RGBDPatchFeatureCalculator<KeypointType, DescriptorType>
{
  //#################### TYPEDEFS ####################
public:
  using typename RGBDPatchFeatureCalculator<KeypointType, DescriptorType>::KeypointImage;
  using typename RGBDPatchFeatureCalculator<KeypointType, DescriptorType>::DescriptorImage;

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
                               KeypointImage *keypointsImage, DescriptorImage *featuresImage, const Matrix4f &cameraPose) const;
};

}

#endif
