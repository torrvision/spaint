/**
 * spaint: SpaintEngine.cpp
 */

#include "main/SpaintEngine.h"

#include <Engine/ImageSourceEngine.h>
#ifdef WITH_OPENNI
#include <Engine/OpenNIEngine.h>
#endif
using namespace InfiniTAM::Engine;

namespace spaint {

//#################### CONSTRUCTORS ####################

#ifdef WITH_OPENNI
SpaintEngine::SpaintEngine(const std::string& calibrationFilename, const boost::optional<std::string>& openNIDeviceURI, const ITMLibSettings& settings)
: m_settings(settings)
{
  m_imageSourceEngine.reset(new OpenNIEngine(calibrationFilename.c_str(), openNIDeviceURI ? openNIDeviceURI->c_str() : NULL));
  // TODO
}
#endif

SpaintEngine::SpaintEngine(const std::string& calibrationFilename, const std::string& rgbImageMask, const std::string& depthImageMask, const ITMLibSettings& settings)
: m_settings(settings)
{
  m_imageSourceEngine.reset(new ImageFileReader(calibrationFilename.c_str(), rgbImageMask.c_str(), depthImageMask.c_str()));
  // TODO
}

}
