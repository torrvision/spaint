/**
 * spaint: ImageProcessor.cpp
 */

#include "imageprocessing/interface/ImageProcessor.h"

namespace spaint {

//#################### DESTRUCTOR ####################

ImageProcessor::~ImageProcessor() {}

//#################### PROTECTED STATIC MEMBER FUNCTIONS ####################

Vector2i ImageProcessor::image_size(const AFArray_CPtr& img)
{
  return Vector2i(img->dims(1), img->dims(0));
}

Vector2i ImageProcessor::image_size(const ITMFloatImage_CPtr& img)
{
  return img->noDims;
}

}
