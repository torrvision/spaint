/**
 * spaint: ImageProcessorFactory.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "imageprocessing/ImageProcessorFactory.h"

#include "imageprocessing/cpu/ImageProcessor_CPU.h"

#ifdef WITH_CUDA
#include "imageprocessing/cuda/ImageProcessor_CUDA.h"
#endif

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

ImageProcessor_CPtr ImageProcessorFactory::make_image_processor(DeviceType deviceType)
{
  ImageProcessor_CPtr imageProcessor;

  if(deviceType == DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    imageProcessor.reset(new ImageProcessor_CUDA);
#else
    // This should never happen as things stand - we set deviceType to DEVICE_CPU to false if CUDA support isn't available.
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    imageProcessor.reset(new ImageProcessor_CPU);
  }

  return imageProcessor;
}

}
