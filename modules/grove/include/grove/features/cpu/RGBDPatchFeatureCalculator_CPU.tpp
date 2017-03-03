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
void RGBDPatchFeatureCalculator_CPU<KeypointType,DescriptorType>::compute_keypoints_and_features(const ITMUChar4Image *rgbImage, const ITMFloatImage *depthImage,
                                                                                                 const Matrix4f& cameraPose, const Vector4f& intrinsics,
                                                                                                 KeypointsImage *keypointsImage, DescriptorsImage *descriptorsImage) const
{
  // Check that the input images are valid and compute the output dimensions.
  const Vector2i outSize = this->compute_output_dims(rgbImage, depthImage);

  // Ensure the output images are the right size (typically this only
  // happens once per program run if the images are properly cached).
  keypointsImage->ChangeDims(outSize);
  descriptorsImage->ChangeDims(outSize);

  const Vector4i *depthOffsets = this->m_depthOffsets->GetData(MEMORYDEVICE_CPU);
  const float *depths = depthImage ? depthImage->GetData(MEMORYDEVICE_CPU) : NULL;
  const Vector2i inSize = depthImage->noDims;
  const Vector4u *rgb = rgbImage ? rgbImage->GetData(MEMORYDEVICE_CPU): NULL;
  const uchar *rgbChannels = this->m_rgbChannels->GetData(MEMORYDEVICE_CPU);
  const Vector4i *rgbOffsets = this->m_rgbOffsets->GetData(MEMORYDEVICE_CPU);

  KeypointType *keypoints = keypointsImage->GetData(MEMORYDEVICE_CPU);
  DescriptorType *descriptors = descriptorsImage->GetData(MEMORYDEVICE_CPU);

  // For each pixel in the RGBD image:
#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int yOut = 0; yOut < outSize.height; ++yOut)
  {
    for(int xOut = 0; xOut < outSize.width; ++xOut)
    {
      const Vector2i xyOut(xOut, yOut);
      const Vector2i xyIn = xyOut * this->m_featureStep;

      // Compute the keypoint for the pixel.
      compute_keypoint(xyIn, xyOut, inSize, outSize, rgb, depths, cameraPose, intrinsics, keypoints);

      // If there is a depth image available and any depth features need to be computed for the keypoint, compute them.
      if(depths && this->m_depthFeatureCount > 0)
      {
        compute_depth_features(
          xyIn, xyOut, inSize, outSize, depths, depthOffsets, keypoints,
          this->m_depthFeatureCount, this->m_depthFeatureOffset,
          this->m_normaliseDepth, descriptors
        );
      }

      // If there is a colour image available and any colour features need to be computed for the keypoint, compute them.
      if(rgb && this->m_rgbFeatureCount > 0)
      {
        compute_colour_features(
          xyIn, xyOut, inSize, outSize, rgb, depths, rgbOffsets, rgbChannels,
          keypoints, this->m_rgbFeatureCount, this->m_rgbFeatureOffset,
          this->m_normaliseRgb, descriptors
        );
      }
    }
  }
}

}
