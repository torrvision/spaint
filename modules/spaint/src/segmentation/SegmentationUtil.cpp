/**
 * spaint: SegmentationUtil.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "segmentation/SegmentationUtil.h"

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

ITMUCharImage_Ptr SegmentationUtil::invert_mask(const ITMUCharImage_CPtr& mask)
{
  ITMUCharImage_Ptr invertedMask(new ITMUCharImage(mask->noDims, true, false));

  const uchar *maskPtr = mask->GetData(MEMORYDEVICE_CPU);
  uchar *invertedMaskPtr = invertedMask->GetData(MEMORYDEVICE_CPU);
  int pixelCount = static_cast<int>(mask->dataSize);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < pixelCount; ++i)
  {
    invertedMaskPtr[i] = maskPtr[i] ? 0 : 255;
  }

  return invertedMask;
}

}
