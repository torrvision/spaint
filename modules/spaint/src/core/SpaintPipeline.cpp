/**
 * spaint: SpaintPipeline.cpp
 */

#include "core/SpaintPipeline.h"

#ifdef WITH_OPENNI
#include <Engine/OpenNIEngine.h>
#endif
#include <ITMLib/Engine/ITMRenTracker.cpp>
#include <ITMLib/Engine/DeviceSpecific/CPU/ITMRenTracker_CPU.cpp>
#include <ITMLib/Engine/DeviceSpecific/CPU/ITMSceneReconstructionEngine_CPU.cpp>
#include <ITMLib/Engine/DeviceSpecific/CPU/ITMSwappingEngine_CPU.cpp>
using namespace InfiniTAM::Engine;

namespace spaint {

//#################### CONSTRUCTORS ####################

#ifdef WITH_OPENNI
SpaintPipeline::SpaintPipeline(const std::string& calibrationFilename, const boost::shared_ptr<std::string>& openNIDeviceURI, const Settings_CPtr& settings)
{
  m_imageSourceEngine.reset(new OpenNIEngine(calibrationFilename.c_str(), openNIDeviceURI ? openNIDeviceURI->c_str() : NULL));
  initialise(settings);
}
#endif

SpaintPipeline::SpaintPipeline(const std::string& calibrationFilename, const std::string& rgbImageMask, const std::string& depthImageMask, const Settings_CPtr& settings)
{
  m_imageSourceEngine.reset(new ImageFileReader(calibrationFilename.c_str(), rgbImageMask.c_str(), depthImageMask.c_str()));
  initialise(settings);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

SpaintModel_CPtr SpaintPipeline::get_model() const
{
  return m_model;
}

SpaintRaycaster_CPtr SpaintPipeline::get_raycaster() const
{
  return m_raycaster;
}

void SpaintPipeline::process_frame()
{
  if(!m_imageSourceEngine->hasMoreImages()) return;

  const SpaintModel::TrackingState_Ptr& trackingState = m_model->get_tracking_state();
  SpaintModel::View_Ptr& view = m_model->get_view();

  // Get the next frame.
  ITMView *newView = view.get();
  m_imageSourceEngine->getImages(m_inputRGBImage.get(), m_inputRawDepthImage.get());
  m_viewBuilder->UpdateView(&newView, m_inputRGBImage.get(), m_inputRawDepthImage.get());
  view.reset(newView);

  // Track the camera (we can only do this once we've started reconstructing the model because we need something to track against).
  if(m_reconstructionStarted) m_trackingController->Track(trackingState.get(), view.get());

  // Run the dense mapper.
  m_denseMapper->ProcessFrame(view.get(), trackingState.get());

  // Raycast from the live camera position to prepare for tracking in the next frame.
  m_trackingController->Prepare(trackingState.get(), view.get());

  m_reconstructionStarted = true;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void SpaintPipeline::initialise(const Settings_CPtr& settings)
{
  // Make sure that we're not trying to run on the GPU if CUDA support isn't enabled.
#ifndef WITH_CUDA
  if(settings.deviceType == ITMLibSettings::DEVICE_CUDA)
  {
    std::cerr << "[spaint] CUDA support unavailable, reverting to the CPU implementation of InfiniTAM\n";
    settings.deviceType = ITMLibSettings::DEVICE_CPU;
  }
#endif

  // Determine the RGB and depth image sizes.
  Vector2i rgbImageSize = m_imageSourceEngine->getRGBImageSize();
  Vector2i depthImageSize = m_imageSourceEngine->getDepthImageSize();
  if(depthImageSize.x == -1 || depthImageSize.y == -1) depthImageSize = rgbImageSize;

  // Set up the RGB and raw depth images into which input is to be read each frame.
  m_inputRGBImage.reset(new ITMUChar4Image(rgbImageSize, true, true));
  m_inputRawDepthImage.reset(new ITMShortImage(depthImageSize, true, true));

  // Set up the spaint model.
  m_model.reset(new SpaintModel(settings, rgbImageSize, depthImageSize, TrackingState_Ptr(m_trackingController->BuildTrackingState())));

  // Set up the InfiniTAM engines and view builder.
  const ITMRGBDCalib *calib = &m_imageSourceEngine->calib;
  if(settings->deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    // Use the GPU implementation of InfiniTAM.
    m_lowLevelEngine.reset(new ITMLowLevelEngine_CUDA);
    m_viewBuilder.reset(new ITMViewBuilder_CUDA(calib));
#else
    // This should never happen as things stand - we set deviceType to DEVICE_CPU to false if CUDA support isn't available.
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    // Use the CPU implementation of InfiniTAM.
    m_lowLevelEngine.reset(new ITMLowLevelEngine_CPU);
    m_viewBuilder.reset(new ITMViewBuilder_CPU(calib));
  }

  // Set up the raycaster.
  m_raycaster.reset(new SpaintRaycaster(m_model));

  // Note: The trackers can only be run once reconstruction has started.
  m_reconstructionStarted = false;
}

}
