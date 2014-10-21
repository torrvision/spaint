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
SpaintEngine::SpaintEngine(const std::string& calibrationFilename, const spaint::shared_ptr<std::string>& openNIDeviceURI, const ITMLibSettings& settings)
: m_settings(settings)
{
  m_imageSourceEngine.reset(new OpenNIEngine(calibrationFilename.c_str(), openNIDeviceURI ? openNIDeviceURI->c_str() : NULL));
  initialise();
}
#endif

SpaintEngine::SpaintEngine(const std::string& calibrationFilename, const std::string& rgbImageMask, const std::string& depthImageMask, const ITMLibSettings& settings)
: m_settings(settings)
{
  m_imageSourceEngine.reset(new ImageFileReader(calibrationFilename.c_str(), rgbImageMask.c_str(), depthImageMask.c_str()));
  initialise();
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void SpaintEngine::initialise()
{
  // Make sure that we're not trying to run on the GPU if CUDA support isn't enabled.
#ifndef WITH_CUDA
  std::cerr << "[spaint] CUDA support unavailable, reverting to the CPU implementation of InfiniTAM\n";
  m_settings.useGPU = false;
#endif

  // Determine the RGB and depth image sizes.
  Vector2i rgbImageSize = m_imageSourceEngine->getRGBImageSize(), depthImageSize = m_imageSourceEngine->getDepthImageSize();
  if(depthImageSize.x == -1 || depthImageSize.y == -1) depthImageSize = rgbImageSize;

  // Set up the scene.
  m_scene.reset(new Scene(&m_settings.sceneParams, m_settings.useSwapping, m_settings.useGPU));

  // Set up the initial tracking state.
  switch(m_settings.trackerType)
  {
    case ITMLibSettings::TRACKER_ICP:
      m_trackingState.reset(new ITMTrackingState(depthImageSize, m_settings.useGPU));
      break;
    case ITMLibSettings::TRACKER_COLOR:
      m_trackingState.reset(new ITMTrackingState(rgbImageSize, m_settings.useGPU));
      break;
    default:
      // This should never happen.
      throw std::runtime_error("SpaintEngine::initialise(): Unknown tracker type");
  }

  m_trackingState->pose_d->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

  // TODO: Set up the initial view(?)

  if(m_settings.useGPU)
  {
#ifdef WITH_CUDA
    // Use the GPU-based InfiniTAM engines.
    // TODO
#else
    // This should never happen as things stand - we set useGPU to false if CUDA support isn't available.
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    // Use the CPU-based InfiniTAM engines.
    // TODO
  }
}

}
