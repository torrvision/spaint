/**
 * spaint: PNGUtil.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "imageprocessing/PNGUtil.h"

#include <stdexcept>

#include <boost/thread.hpp>

#include <lodepng.h>

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

PNGUtil::ITMUChar4Image_Ptr PNGUtil::load_rgba_image(const std::string& path)
{
  std::vector<unsigned char> buffer;
  lodepng::load_file(buffer, path);
  if(buffer.empty()) throw std::runtime_error("Could not load PNG image from '" + path + "'");
  return decode_rgba_png(buffer, path);
}

void PNGUtil::save_image(const ITMUChar4Image_CPtr& image, const std::string& path)
{
  std::vector<unsigned char> buffer;
  encode_png(image, buffer);
  lodepng::save_file(buffer, path);
}

void PNGUtil::save_image_on_thread(const ITMUChar4Image_CPtr& image, const std::string& path)
{
  boost::thread t(&save_image, image, path);
  t.detach();
}

void PNGUtil::save_image_on_thread(const ITMUChar4Image_CPtr& image, const boost::filesystem::path& path)
{
  save_image_on_thread(image, path.string());
}

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

PNGUtil::ITMUChar4Image_Ptr PNGUtil::decode_rgba_png(const std::vector<unsigned char>& buffer, const std::string& path)
{
  // Decode the PNG.
  std::vector<unsigned char> data;
  unsigned int width, height;
  if(lodepng::decode(data, width, height, buffer) != 0)
  {
    throw std::runtime_error("Failed to decode PNG from '" + path + "'");
  }

  // Construct the image.
  ITMUChar4Image_Ptr image(new ITMUChar4Image(Vector2i(width, height), true, true));
  const int pixelCount = width * height;
  const unsigned char *src = &data[0];
  Vector4u *dest = image->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < pixelCount; ++i)
  {
    Vector4u& pixel = dest[i];
    int offset = i * 4;
    pixel.r = src[offset];
    pixel.g = src[offset + 1];
    pixel.b = src[offset + 2];
    pixel.a = src[offset + 3];
  }

  return image;
}

void PNGUtil::encode_png(const ITMUChar4Image_CPtr& image, std::vector<unsigned char>& buffer)
{
  const int pixelCount = static_cast<int>(image->dataSize);
  std::vector<unsigned char> data(pixelCount * 4);
  const Vector4u *src = image->GetData(MEMORYDEVICE_CPU);
  unsigned char *dest = &data[0];

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < pixelCount; ++i)
  {
    const Vector4u& pixel = src[i];
    int offset = i * 4;
    dest[offset] = pixel.r;
    dest[offset + 1] = pixel.g;
    dest[offset + 2] = pixel.b;
    dest[offset + 3] = pixel.a;
  }

  lodepng::encode(buffer, &data[0], image->noDims.x, image->noDims.y);
}

}
