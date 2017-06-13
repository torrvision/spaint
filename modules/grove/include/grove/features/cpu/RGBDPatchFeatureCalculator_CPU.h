/**
 * grove: RGBDPatchFeatureCalculator_CPU.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_RGBDPATCHFEATURECALCULATOR_CPU
#define H_GROVE_RGBDPATCHFEATURECALCULATOR_CPU

#include "../interface/RGBDPatchFeatureCalculator.h"

namespace grove {

/**
 * \brief An instance of this class can be used to compute features based on depth and colour differences in RGBD images using the CPU.
 *
 * The features are computed as described in "Exploiting Uncertainty in Regression Forests for Accurate Camera Relocalization".
 *
 * \param KeypointType    The type of keypoint computed by this class.
 * \param DescriptorType  The type of descriptor computed by this class.
 */
template <typename KeypointType, typename DescriptorType>
class RGBDPatchFeatureCalculator_CPU : public RGBDPatchFeatureCalculator<KeypointType,DescriptorType>
{
  //#################### USINGS ####################
public:
  using typename RGBDPatchFeatureCalculator<KeypointType,DescriptorType>::DescriptorsImage;
  using typename RGBDPatchFeatureCalculator<KeypointType,DescriptorType>::KeypointsImage;

  //#################### CONSTRUCTORS ####################
private:
  /**
   * \brief Constructs a CPU-based RGBD patch feature calculator.
   *
   * Note: This is private to force clients to make use of FeatureCalculatorFactory, which knows the correct values to use for the arguments.
   *
   * \param depthAdaptive        Whether or not to compute the depth-normalised version of the features.
   * \param depthDifferenceType  The type of difference to use to compute depth features.
   * \param depthFeatureCount    The number of features to compute from the depth image.
   * \param depthFeatureOffset   The offset in the descriptor after which we store the depth features.
   * \param depthMinRadius       The minimum radius used to generate depth offsets.
   * \param depthMaxRadius       The maximum radius used to generate depth offsets.
   * \param rgbDifferenceType    The type of difference to use to compute RGB features.
   * \param rgbFeatureCount      The number of features to compute from the RGB image.
   * \param rgbFeatureOffset     The offset in the descriptor after which we store the RGB features.
   * \param rgbMinRadius         The minimum radius used to generate colour offsets.
   * \param rgbMaxRadius         The maximum radius used to generate colour offsets.
   *
   * \throws std::invalid_argument If depthFeatureCount + rgbFeatureCount > DescriptorType::FEATURE_COUNT, or if the offsets would cause out-of-bounds access.
   */
  RGBDPatchFeatureCalculator_CPU(bool depthAdaptive, RGBDPatchFeatureDifferenceType depthDifferenceType, uint32_t depthFeatureCount,
                                 uint32_t depthFeatureOffset, uint32_t depthMinRadius, uint32_t depthMaxRadius,
                                 RGBDPatchFeatureDifferenceType rgbDifferenceType, uint32_t rgbFeatureCount,
                                 uint32_t rgbFeatureOffset, uint32_t rgbMinRadius, uint32_t rgbMaxRadius);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void compute_keypoints_and_features(const ITMUChar4Image *rgbImage, const ITMFloatImage *depthImage,
                                              const Matrix4f& cameraPose, const Vector4f& intrinsics,
                                              KeypointsImage *keypointsImage, DescriptorsImage *descriptorsImage) const;

  //#################### FRIENDS ####################

  friend struct FeatureCalculatorFactory;
};

}

#endif
