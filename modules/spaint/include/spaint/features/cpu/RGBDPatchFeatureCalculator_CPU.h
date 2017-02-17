/**
 * spaint: RGBDPatchFeatureCalculator_CPU.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_SPAINT_RGBDPATCHFEATURECALCULATOR_CPU
#define H_SPAINT_RGBDPATCHFEATURECALCULATOR_CPU

#include "../interface/RGBDPatchFeatureCalculator.h"

namespace spaint
{

/**
 * \brief An instance of this class can be used to compute features based on
 *        depth and colour differences in RGBD images using the CPU.
 *
 * The features are computed as described by Valentin et al. in "Exploiting
 * Uncertainty in Regression Forests for Accurate Camera Relocalization".
 *
 * \param KeypointType    The type of keypoints computed by this class.
 * \param DescriptorType  he type of descriptors computed by this class.
 */
template<typename KeypointType, typename DescriptorType>
class RGBDPatchFeatureCalculator_CPU : public RGBDPatchFeatureCalculator<KeypointType, DescriptorType>
{
  //#################### TYPEDEFS ####################
public:
  using typename RGBDPatchFeatureCalculator<KeypointType, DescriptorType>::KeypointImage;
  using typename RGBDPatchFeatureCalculator<KeypointType, DescriptorType>::DescriptorImage;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a CPU-based RGBD patch feature calculator.
   *
   *
   * Note: meant to be instantiated with FeatureCalculatorFactory, to get correct default parameters.
   *
   * \param depthAdaptive      Whether to compute the depth-normalised version of the features.
   * \param depthFeatureCount  The number of features computed from the depth image.
   * \param depthFeatureOffset The offset in the output descriptor after which we store the depthFeatureCount depth features.
   * \param rgbFeatureCount    The number of features computed from the RGB image.
   * \param rgbFeatureOffset   The offset in the output descriptor after which we store the rgbFeatureCount colour features.
   *
   * \throws std::invalid_argument if depthFeatureCount + rgbFeatureCount > DescriptorType::FEATURE_COUNT or if the offsets cause out of bounds access.
   */
  RGBDPatchFeatureCalculator_CPU(bool depthAdaptive,
                                 uint32_t depthFeatureCount,
                                 uint32_t depthFeatureOffset,
                                 uint32_t rgbFeatureCount,
                                 uint32_t rgbFeatureOffset);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void compute_feature(const ITMUChar4Image *rgbImage, const ITMFloatImage *depthImage, const Vector4f &intrinsics,
                               KeypointImage *keypointsImage, DescriptorImage *featuresImage, const Matrix4f &cameraPose) const;
};

}

#endif
