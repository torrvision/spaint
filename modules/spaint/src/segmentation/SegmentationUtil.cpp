/**
 * spaint: SegmentationUtil.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "segmentation/SegmentationUtil.h"

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

SegmentationUtil::ITMUChar4Image_Ptr SegmentationUtil::apply_mask(const ITMUCharImage_CPtr& mask, const ITMUChar4Image_CPtr& image)
{
  ITMUChar4Image_Ptr maskedImage(new ITMUChar4Image(image->noDims, true, false));

  const uchar *maskPtr = mask->GetData(MEMORYDEVICE_CPU);
  const Vector4u *imagePtr = image->GetData(MEMORYDEVICE_CPU);
  Vector4u *maskedImagePtr = maskedImage->GetData(MEMORYDEVICE_CPU);

  int pixelCount = static_cast<int>(image->dataSize);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < pixelCount; ++i)
  {
    maskedImagePtr[i] = maskPtr[i] ? imagePtr[i] : Vector4u((uchar)0);
  }

  return maskedImage;
}

}
