/**
 * grove: RGBDPatchFeatureCalculator_CPU.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "features/cpu/RGBDPatchFeatureCalculator_CPU.h"

#include "features/shared/RGBDPatchFeatureCalculator_Shared.h"

namespace grove {

//#################### CONSTRUCTORS ####################

template <typename KeypointType, typename DescriptorType>
RGBDPatchFeatureCalculator_CPU<KeypointType,DescriptorType>::RGBDPatchFeatureCalculator_CPU(bool depthAdaptive, uint32_t depthFeatureCount, uint32_t depthFeatureOffset,
                                                                                            uint32_t rgbFeatureCount, uint32_t rgbFeatureOffset)
: RGBDPatchFeatureCalculator<KeypointType,DescriptorType>(depthAdaptive, depthFeatureCount, depthFeatureOffset, rgbFeatureCount, rgbFeatureOffset)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

template <typename KeypointType, typename DescriptorType>
void RGBDPatchFeatureCalculator_CPU<KeypointType,DescriptorType>::compute_feature(const ITMUChar4Image *rgbImage, const ITMFloatImage *depthImage,
                                                                                  const Matrix4f& cameraPose, const Vector4f& intrinsics,
                                                                                  KeypointsImage *keypointsImage, DescriptorsImage *descriptorsImage) const
{
  // Check that the input images are valid.
  this->check_input_images(rgbImage, depthImage);

  // Ensure the output images are the right size (typically this only
  // happens once per program run if the images are properly cached).
  // They should have one pixel for each element of the sampling grid.
  const Vector2i outDims = depthImage->noDims / this->m_featureStep;
  keypointsImage->ChangeDims(outDims);
  descriptorsImage->ChangeDims(outDims);

  const float *depth = depthImage->GetData(MEMORYDEVICE_CPU);
  const Vector4i *depthOffsets = this->m_depthOffsets->GetData(MEMORYDEVICE_CPU);
  const Vector2i inDims = depthImage->noDims;
  const Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);
  const uchar *rgbChannels = this->m_rgbChannels->GetData(MEMORYDEVICE_CPU);
  const Vector4i *rgbOffsets = this->m_rgbOffsets->GetData(MEMORYDEVICE_CPU);

  KeypointType *keypoints = keypointsImage->GetData(MEMORYDEVICE_CPU);
  DescriptorType *features = descriptorsImage->GetData(MEMORYDEVICE_CPU);

  // For each pixel in the RGBD image:
#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int yOut = 0; yOut < outDims.height; ++yOut)
  {
    for(int xOut = 0; xOut < outDims.width; ++xOut)
    {
      const Vector2i xyOut(xOut, yOut);
      const Vector2i xyIn = xyOut * this->m_featureStep;

      // Compute the keypoint for the pixel.
      compute_keypoint(keypoints, rgb, depth, intrinsics, inDims, outDims, xyIn, xyOut, cameraPose);

      // If there is a depth image available and any depth features need to be computed for the keypoint, compute them.
      if(depth && this->m_depthFeatureCount > 0)
      {
        compute_depth_patch_feature(
          keypoints, features, depth, depthOffsets, inDims, outDims,
          intrinsics, cameraPose, this->m_normaliseDepth, xyIn, xyOut,
          this->m_depthFeatureCount, this->m_depthFeatureOffset
        );
      }

      // If there is a colour image available and any colour features need to be computed for the keypoint, compute them.
      if(rgb && this->m_rgbFeatureCount > 0)
      {
        compute_colour_patch_feature(
          keypoints, features, rgb, depth, rgbOffsets, rgbChannels, inDims, outDims,
          this->m_normaliseRgb, xyIn, xyOut, this->m_rgbFeatureCount, this->m_rgbFeatureOffset
        );
      }
    }
  }
}

}
