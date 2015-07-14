/**
 * spaint: ImageProcessor.cpp
 */

#include "imageprocessing/interface/ImageProcessor.h"

namespace spaint {

//#################### DESTRUCTOR ####################

ImageProcessor::~ImageProcessor() {}

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

Vector2i ImageProcessor::image_size(const AFArray_CPtr& img)
{
  return Vector2i(img->dims(1), img->dims(0));
}

void ImageProcessor::median_filter(const ITMUChar4Image_CPtr& inputImage, const ITMUChar4Image_Ptr& outputImage) const
try
{
  AFArray_Ptr afImage(new af::array(inputImage->noDims.y, inputImage->noDims.x, 4, u8));
  copy_itm_to_af(inputImage, afImage);
  *afImage = af::medfilt(*afImage, 5, 5);
  copy_af_to_itm(afImage, outputImage);
}
catch(af::exception&)
{
  // Prevent crashes when median filtering fails.
}

}
