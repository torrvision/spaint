/**
 * raflvis: PaletteGenerator.cpp
 */

#include "PaletteGenerator.h"

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

std::map<std::string,cv::Scalar> PaletteGenerator::generate_basic_rgba_palette()
{
  const int alpha = 255;

  std::map<std::string,cv::Scalar> result = boost::assign::map_list_of
    ("Black",cv::Scalar(0,0,0,alpha))
    ("White",cv::Scalar(255,255,255,alpha))
    ("Red",cv::Scalar(255,0,0,alpha))
    ("Lime",cv::Scalar(0,255,0,alpha))
    ("Blue",cv::Scalar(0,0,255,alpha))
    ("Yellow",cv::Scalar(255,255,0,alpha))
    ("Cyan",cv::Scalar(0,255,255,alpha))
    ("Magenta",cv::Scalar(255,0,255,alpha))
    ("Silver",cv::Scalar(192,192,192,alpha))
    ("Gray",cv::Scalar(128,128,128,alpha))
    ("Maroon",cv::Scalar(128,0,0,alpha))
    ("Olive",cv::Scalar(128,128,0,alpha))
    ("Green",cv::Scalar(0,128,0,alpha))
    ("Purple",cv::Scalar(128,0,128,alpha))
    ("Teal",cv::Scalar(0,128,128,alpha))
    ("Navy",cv::Scalar(0,0,128,alpha));

  return result;
}

