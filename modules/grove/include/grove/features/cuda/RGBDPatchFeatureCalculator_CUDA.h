/**
 * grove: RGBDPatchFeatureCalculator_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_RGBDPATCHFEATURECALCULATOR_CUDA
#define H_GROVE_RGBDPATCHFEATURECALCULATOR_CUDA

#include "../interface/RGBDPatchFeatureCalculator.h"

namespace grove {

/**
 * \brief An instance of this class can be used to compute features based on depth and colour differences in RGBD images using CUDA.
 *
 * The features are computed as described by Valentin et al. in "Exploiting Uncertainty in Regression Forests for Accurate Camera Relocalization".
 *
 * \param KeypointType    The type of keypoint computed by this class.
 * \param DescriptorType  The type of descriptor computed by this class.
 */
template <typename KeypointType, typename DescriptorType>
class RGBDPatchFeatureCalculator_CUDA : public RGBDPatchFeatureCalculator<KeypointType,DescriptorType>
{
  //#################### TYPEDEFS ####################
public:
  using typename RGBDPatchFeatureCalculator<KeypointType,DescriptorType>::DescriptorsImage;
  using typename RGBDPatchFeatureCalculator<KeypointType,DescriptorType>::KeypointsImage;

  //#################### CONSTRUCTORS ####################
private:
  /**
   * \brief Constructs a CUDA-based RGBD patch feature calculator.
   *
   * Note: This is private to force clients to make use of FeatureCalculatorFactory, which knows the correct values to use for the arguments.
   *
   * \param depthAdaptive        Whether or not to compute the depth-normalised version of the features.
   * \param depthDifferenceType  The kind of differencing to use for the depth part of the descriptor.
   * \param depthFeatureCount    The number of features to compute from the depth image.
   * \param depthFeatureOffset   The offset in the descriptor after which we store the depth features.
   * \param depthMinRadius       The minimum radius used to generate depth offsets.
   * \param depthMaxRadius       The maximum radius used to generate depth offsets.
   * \param rgbDifferenceType    The kind of differencing to use for the colour part of the descriptor.
   * \param rgbFeatureCount      The number of features to compute from the RGB image.
   * \param rgbFeatureOffset     The offset in the descriptor after which we store the colour features.
   * \param rgbMinRadius         The minimum radius used to generate colour offsets.
   * \param rgbMaxRadius         The maximum radius used to generate colour offsets.
   *
   * \throws std::invalid_argument If depthFeatureCount + rgbFeatureCount > DescriptorType::FEATURE_COUNT, or if the offsets cause out-of-bounds access.
   */
  RGBDPatchFeatureCalculator_CUDA(bool depthAdaptive,
                                  RGBDPatchFeatureCalculatorDifferenceType depthDifferenceType,
                                  uint32_t depthFeatureCount,
                                  uint32_t depthFeatureOffset,
                                  uint32_t depthMinRadius,
                                  uint32_t depthMaxRadius,
                                  RGBDPatchFeatureCalculatorDifferenceType rgbDifferenceType,
                                  uint32_t rgbFeatureCount,
                                  uint32_t rgbFeatureOffset,
                                  uint32_t rgbMinRadius,
                                  uint32_t rgbMaxRadius);

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
