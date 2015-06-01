/**
 * spaint: ImageProcessor.cpp
 */

#include "imageprocessing/interface/ImageProcessor.h"

namespace spaint {

//#################### DESTRUCTOR ####################

ImageProcessor::~ImageProcessor() {}

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

int ImageProcessor::column_major_index_from_row_major_index(int rowMajorIndex, int width, int height)
{
  int row = rowMajorIndex / width;
  int col = rowMajorIndex % width;
  return col * height + row;
}

//#################### PROTECTED STATIC MEMBER FUNCTIONS ####################

Vector2i ImageProcessor::image_size(const AFImage_CPtr& img)
{
  return Vector2i(img->dims(1), img->dims(0));
}

Vector2i ImageProcessor::image_size(const ITMFloatImage_CPtr& img)
{
  return img->noDims;
}

}
