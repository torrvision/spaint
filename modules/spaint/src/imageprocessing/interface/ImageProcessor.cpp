/**
 * spaint: ImageProcessor.cpp
 */

#include "imageprocessing/interface/ImageProcessor.h"

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

int ImageProcessor::column_major_index_from_row_major_index(int rowMajorIndex, int width, int height)
{
  int row = rowMajorIndex / width;
  int col = rowMajorIndex % width;
  return col * height + row;
}

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

void ImageProcessor::check_equal(const std::vector<int>& a, const std::vector<int>& b)
{
  if(a != b) throw std::runtime_error("The image dimensions are not equal");
}

std::vector<int> ImageProcessor::image_size(const af::array *img)
{
  return std::vector<int>(img->dims(0), img->dims(1));
}

std::vector<int> ImageProcessor::image_size(const ITMFloatImage* img)
{
  return std::vector<int>(img->noDims.y, img->noDims.x);
}

}

