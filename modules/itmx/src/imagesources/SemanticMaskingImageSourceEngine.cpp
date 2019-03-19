/**
 * itmx: SemanticMaskingImageSourceEngine.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2019. All rights reserved.
 */

#include "imagesources/SemanticMaskingImageSourceEngine.h"
using namespace ITMLib;

#include <boost/assign/list_of.hpp>

namespace itmx {

//#################### CONSTRUCTORS ####################

SemanticMaskingImageSourceEngine::SemanticMaskingImageSourceEngine(ImageSourceEngine *normalSource, ImageSourceEngine *semanticSource, MaskingType maskingType,
                                                                   const std::set<std::string>& labelsToMask, const std::string& labelSet)
: m_maskingType(maskingType), m_normalSource(normalSource), m_semanticSource(semanticSource)
{
  // Construct a map from semantic labels to colours.
  // TODO: We should allow other label sets to be used as well.
  if(labelSet == "cityscapes")
  {
    // See: https://github.com/mcordts/cityscapesScripts/blob/master/cityscapesscripts/helpers/labels.py
    m_labelToColourMap = boost::assign::map_list_of
      ("unlabeled",            Vector3u(0,0,0))
      ("ego vehicle",          Vector3u(0,0,0))
      ("rectification border", Vector3u(0,0,0))
      ("out of roi",           Vector3u(0,0,0))
      ("static",               Vector3u(0,0,0))
      ("dynamic",              Vector3u(111,74,0))
      ("ground",               Vector3u(81,0,81))
      ("road",                 Vector3u(128,64,128))
      ("sidewalk",             Vector3u(244,35,232))
      ("parking",              Vector3u(250,170,160))
      ("rail track",           Vector3u(230,150,140))
      ("building",             Vector3u(70,70,70))
      ("wall",                 Vector3u(102,102,156))
      ("fence",                Vector3u(190,153,153))
      ("guard rail",           Vector3u(180,165,180))
      ("bridge",               Vector3u(150,100,100))
      ("tunnel",               Vector3u(150,120,90))
      ("pole",                 Vector3u(153,153,153))
      ("polegroup",            Vector3u(153,153,153))
      ("traffic light",        Vector3u(250,170,30))
      ("traffic sign",         Vector3u(220,220,0))
      ("vegetation",           Vector3u(107,142,35))
      ("terrain",              Vector3u(152,251,152))
      ("sky",                  Vector3u(70,130,180))
      ("person",               Vector3u(220,20,60))
      ("rider",                Vector3u(255,0,0))
      ("car",                  Vector3u(0,0,142))
      ("truck",                Vector3u(0,0,70))
      ("bus",                  Vector3u(0,60,100))
      ("caravan",              Vector3u(0,0,90))
      ("trailer",              Vector3u(0,0,110))
      ("train",                Vector3u(0,80,100))
      ("motorcycle",           Vector3u(0,0,230))
      ("bicycle",              Vector3u(119,11,32))
      ("license plate",        Vector3u(0,0,142))
    .convert_to_container<std::map<std::string,Vector3u> >();
  }
  else throw std::runtime_error("Error: Unknown semantic label set '" + labelSet + "'");

  // Determine which colours to mask out by looking up the labels to mask out in the map.
  m_coloursToMask.reserve(labelsToMask.size());
  for(std::set<std::string>::const_iterator it = labelsToMask.begin(), iend = labelsToMask.end(); it != iend; ++it)
  {
    std::map<std::string,Vector3u>::const_iterator jt = m_labelToColourMap.find(*it);
    if(jt != m_labelToColourMap.end())
    {
      m_coloursToMask.push_back(jt->second);
    }
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

ITMRGBDCalib SemanticMaskingImageSourceEngine::getCalib() const
{
  return m_normalSource->getCalib();
}

Vector2i SemanticMaskingImageSourceEngine::getDepthImageSize() const
{
  return m_normalSource->getDepthImageSize();
}

void SemanticMaskingImageSourceEngine::getImages(ORUChar4Image *rgb, ORShortImage *rawDepth)
{
  if(m_semanticSource->hasMoreImages())
  {
    if(!m_semanticImage) m_semanticImage.reset(new ORUChar4Image(rgb->noDims, true, true));

    m_normalSource->getImages(rgb, rawDepth);
    m_semanticSource->getImages(m_semanticImage.get(), rawDepth);

    const Vector4u zeroColour(0, 0, 0, 0);
    const Vector4u *semanticImage = m_semanticImage->GetData(MEMORYDEVICE_CPU);

  #ifdef WITH_OPENMP
    #pragma omp parallel for
  #endif
    for(int y = 0; y < rgb->noDims.y; ++y)
    {
      for(int x = 0; x < rgb->noDims.x; ++x)
      {
        // FIXME: Make this work even if the RGB and depth images aren't the same size.
        const int offset = y * rgb->noDims.x + x;
        const Vector3u c = semanticImage[offset].toVector3();
        for(size_t i = 0, size = m_coloursToMask.size(); i < size; ++i)
        {
          if(c == m_coloursToMask[i])
          {
            if(m_maskingType != MASK_DEPTH_ONLY) rgb->GetData(MEMORYDEVICE_CPU)[offset] = zeroColour;
            if(m_maskingType != MASK_RGB_ONLY) rawDepth->GetData(MEMORYDEVICE_CPU)[offset] = 0;
          }
        }
      }
    }
  }
  else
  {
    m_normalSource->getImages(rgb, rawDepth);
  }
}

Vector2i SemanticMaskingImageSourceEngine::getRGBImageSize() const
{
  return m_normalSource->getRGBImageSize();
}

bool SemanticMaskingImageSourceEngine::hasMoreImages() const
{
  return m_normalSource->hasMoreImages();
}

}
