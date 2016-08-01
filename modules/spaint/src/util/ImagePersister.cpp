/**
 * spaint: ImagePersister.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "util/ImagePersister.h"

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

ImagePersister::ITMUChar4Image_Ptr ImagePersister::load_rgba_image(const std::string& path, ImageFileType fileType)
{
  // If the image file type wasn't specified, try to deduce it.
  if(fileType == IFT_UNKNOWN) fileType = deduce_image_file_type(path);

  // Load the image in an appropriate way based on its file type.
  switch(fileType)
  {
    case IFT_PNG:
    {
      std::vector<unsigned char> buffer;
      lodepng::load_file(buffer, path);
      if(buffer.empty()) throw std::runtime_error("Could not load PNG image from '" + path + "'");
      return decode_rgba_png(buffer, path);
    }
    default:
    {
      throw std::runtime_error("Could not load image from '" + path + "': unsupported file type");
    }
  }
}

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

ImagePersister::ITMUChar4Image_Ptr ImagePersister::decode_rgba_png(const std::vector<unsigned char>& buffer, const std::string& path)
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

ImagePersister::ImageFileType ImagePersister::deduce_image_file_type(const std::string& path)
{
  return IFT_UNKNOWN;
}

void ImagePersister::encode_png(const ITMUChar4Image_CPtr& image, std::vector<unsigned char>& buffer)
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
