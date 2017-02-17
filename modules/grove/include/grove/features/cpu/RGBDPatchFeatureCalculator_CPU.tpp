/**
 * grove: RGBDPatchFeatureCalculator_CPU.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "features/cpu/RGBDPatchFeatureCalculator_CPU.h"
#include "features/shared/RGBDPatchFeatureCalculator_Shared.h"

namespace grove
{

//#################### CONSTRUCTORS ####################

template<typename KeypointType, typename DescriptorType>
RGBDPatchFeatureCalculator_CPU<KeypointType, DescriptorType>::RGBDPatchFeatureCalculator_CPU(
    bool depthAdaptive,
    uint32_t depthFeatureCount,
    uint32_t depthFeatureOffset,
    uint32_t rgbFeatureCount,
    uint32_t rgbFeatureOffset)
    : RGBDPatchFeatureCalculator<KeypointType, DescriptorType>(
        depthAdaptive, depthFeatureCount, depthFeatureOffset, rgbFeatureCount, rgbFeatureOffset)
{
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

template<typename KeypointType, typename DescriptorType>
void RGBDPatchFeatureCalculator_CPU<KeypointType, DescriptorType>::compute_feature(const ITMUChar4Image *rgbImage, const ITMFloatImage *depthImage,
                                                                                   const Matrix4f& cameraPose, const Vector4f& intrinsics,
                                                                                   KeypointImage *keypointsImage, DescriptorImage *featuresImage) const
{
  // Validate inputs
  this->validate_input_images(rgbImage, depthImage);

  const Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);
  const float *depth = depthImage->GetData(MEMORYDEVICE_CPU);

  const Vector4i *offsetsRgb = this->m_rgbOffsets->GetData(MEMORYDEVICE_CPU);
  const uchar *channelsRgb = this->m_rgbChannels->GetData(MEMORYDEVICE_CPU);
  const Vector4i *offsetsDepth = this->m_depthOffsets->GetData(MEMORYDEVICE_CPU);

  Vector2i inDims = depthImage->noDims;
  // The output images have one pixel per each element of the sampling grid.
  Vector2i outDims(depthImage->noDims.x / this->m_featureStep,
      depthImage->noDims.y / this->m_featureStep);

  // Resize the output images as needed
  // (typically this happens only once per run of the program if the images are properly cached).
  keypointsImage->ChangeDims(outDims);
  featuresImage->ChangeDims(outDims);

  KeypointType *keypoints = keypointsImage->GetData(MEMORYDEVICE_CPU);
  DescriptorType *features = featuresImage->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (int yOut = 0; yOut < outDims.height; ++yOut)
  {
    for (int xOut = 0; xOut < outDims.width; ++xOut)
    {
      const Vector2i xyOut(xOut, yOut);
      const Vector2i xyIn(xOut * this->m_featureStep, yOut * this->m_featureStep);

      // First of all compute keypoints.
      compute_keypoint(keypoints, rgb, depth, intrinsics, inDims, outDims, xyIn, xyOut, cameraPose);

      // Compute depth features if needed.
      if(depth && this->m_depthFeatureCount > 0)
      {
        compute_depth_patch_feature(keypoints, features, depth, offsetsDepth,
            inDims, outDims, intrinsics, cameraPose, this->m_normalizeDepth, xyIn, xyOut,
            this->m_depthFeatureCount, this->m_depthFeatureOffset);
      }

      // Compute colour features.
      if(rgb && this->m_rgbFeatureCount > 0)
      {
        compute_colour_patch_feature(keypoints, features, rgb, depth, offsetsRgb,
            channelsRgb, inDims, outDims, this->m_normalizeRgb, xyIn, xyOut,
            this->m_rgbFeatureCount, this->m_rgbFeatureOffset);
      }
    }
  }
}

}
