/**
 * itmx: RemoteImageSourceEngine.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "imagesources/RemoteImageSourceEngine.h"
using namespace ITMLib;

namespace itmx {

//#################### CONSTRUCTORS ####################

RemoteImageSourceEngine::RemoteImageSourceEngine(const MappingServer_Ptr& mappingServer, int clientID)
: m_clientID(clientID), m_mappingServer(mappingServer)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

ITMRGBDCalib RemoteImageSourceEngine::getCalib() const
{
  return m_mappingServer->get_calib(m_clientID);
}

Vector2i RemoteImageSourceEngine::getDepthImageSize() const
{
  return m_mappingServer->get_depth_image_size(m_clientID);
}

void RemoteImageSourceEngine::getImages(ORUChar4Image *rgb, ORShortImage *rawDepth)
{
  m_mappingServer->get_images(m_clientID, rgb, rawDepth);
}

Vector2i RemoteImageSourceEngine::getRGBImageSize() const
{
  return m_mappingServer->get_rgb_image_size(m_clientID);
}

bool RemoteImageSourceEngine::hasImagesNow() const
{
  return m_mappingServer->has_images_now(m_clientID);
}

bool RemoteImageSourceEngine::hasMoreImages() const
{
  return m_mappingServer->has_more_images(m_clientID);
}

}
