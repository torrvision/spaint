/**
 * spaint: RGBDPatchFeatureCalculator_CPU.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "features/cpu/RGBDPatchFeatureCalculator_CPU.h"
#include "features/shared/RGBDPatchFeatureCalculator_Shared.h"

namespace spaint
{

//#################### PUBLIC MEMBER FUNCTIONS ####################

template<typename KeypointType, typename DescriptorType>
void RGBDPatchFeatureCalculator_CPU<KeypointType, DescriptorType>::compute_feature(
    const ITMUChar4Image *rgbImage, const ITMFloatImage *depthImage,
    const Vector4f &intrinsics, KeypointImage *keypointsImage,
    DescriptorImage *featuresImage, const Matrix4f &cameraPose) const
{
  const Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);
  const float *depth = depthImage->GetData(MEMORYDEVICE_CPU);

  const Vector4i *offsetsRgb = this->m_offsetsRgb->GetData(MEMORYDEVICE_CPU);
  const uchar *channelsRgb = this->m_channelsRgb->GetData(MEMORYDEVICE_CPU);
  const Vector4i *offsetsDepth = this->m_offsetsDepth->GetData(MEMORYDEVICE_CPU);

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

      compute_depth_patch_feature(keypoints, features, depth, offsetsDepth,
          inDims, outDims, intrinsics, cameraPose, this->m_normalizeDepth, xyIn, xyOut,
          this->m_countDepthFeatures, this->m_offsetDepthFeatures);

      if(rgb)
      {
        compute_colour_patch_feature(keypoints, features, rgb, depth, offsetsRgb,
            channelsRgb, inDims, outDims, this->m_normalizeRgb, xyIn, xyOut,
            this->m_countRgbFeatures, this->m_offsetRgbFeatures);
      }
    }
  }
}

}
