/**
 * spaint: SpaintEngine.cpp
 */

#include "main/SpaintEngine.h"

#include <stdexcept>

#include <Engine/ImageSourceEngine.h>
#ifdef WITH_OPENNI
#include <Engine/OpenNIEngine.h>
#endif
#include <ITMLib/Engine/DeviceSpecific/CPU/ITMSceneReconstructionEngine_CPU.cpp>
#include <ITMLib/Engine/DeviceSpecific/CPU/ITMSwappingEngine_CPU.cpp>
#include <ITMLib/Engine/DeviceSpecific/CPU/ITMVisualisationEngine_CPU.cpp>
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

//#################### PUBLIC MEMBER FUNCTIONS ####################

void SpaintEngine::process_frame()
{
  if(!m_imageSourceEngine->hasMoreImages()) return;

  // Get the next frame.
  m_imageSourceEngine->getImages(m_view.get());

  // If we're using the GPU, transfer the relevant images in the frame across to it.
  if(m_settings.useGPU)
  {
    m_view->rgb->UpdateDeviceFromHost();

    switch(m_view->inputImageType)
    {
      case ITMView::InfiniTAM_FLOAT_DEPTH_IMAGE:
        m_view->depth->UpdateDeviceFromHost();
        break;
      case ITMView::InfiniTAM_SHORT_DEPTH_IMAGE:
      case ITMView::InfiniTAM_DISPARITY_IMAGE:
        m_view->rawDepth->UpdateDeviceFromHost();
        break;
      default:
        // This should never happen.
        throw std::runtime_error("Error: SpaintEngine::process_frame: Unknown input image type");
    }
  }

  // If the frame only contains a short depth image or a disparity image, make a proper floating-point depth image from what we have.
  // Note that this will automatically run on either the GPU or the CPU behind the scenes, depending on which mode we're in.
  switch(m_view->inputImageType)
  {
    case ITMView::InfiniTAM_SHORT_DEPTH_IMAGE:
      m_lowLevelEngine->ConvertDepthMMToFloat(m_view->depth, m_view->rawDepth);
      break;
    case ITMView::InfiniTAM_DISPARITY_IMAGE:
      m_lowLevelEngine->ConvertDisparityToDepth(m_view->depth, m_view->rawDepth, &m_view->calib->intrinsics_d, &m_view->calib->disparityCalib);
      break;
    default:
      break;
  }

  // Track the camera (we can only do this once we've started reconstructing the model because we need something to track against).
  if(m_reconstructionStarted) m_tracker->TrackCamera(m_trackingState.get(), m_view.get());

  // TODO
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
  m_trackingState.reset(m_settings.trackerType == ITMLibSettings::TRACKER_ICP ? new ITMTrackingState(depthImageSize, m_settings.useGPU) : new ITMTrackingState(rgbImageSize, m_settings.useGPU));
  m_trackingState->pose_d->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

  // Set up the scene view.
  m_view.reset(new ITMView(m_imageSourceEngine->calib, rgbImageSize, depthImageSize, m_settings.useGPU));

  // Set up the InfiniTAM engines and the tracker.
  if(m_settings.useGPU)
  {
#ifdef WITH_CUDA
    // Use the GPU implementation of InfiniTAM.
    m_lowLevelEngine.reset(new ITMLowLevelEngine_CUDA);
    m_sceneReconstructionEngine.reset(new ITMSceneReconstructionEngine_CUDA<SpaintVoxel,ITMVoxelIndex>);

    if(m_settings.trackerType == ITMLibSettings::TRACKER_ICP)
      m_tracker.reset(new ITMDepthTracker_CUDA(depthImageSize, m_settings.noHierarchyLevels, m_settings.noRotationOnlyLevels, m_settings.depthTrackerICPThreshold, m_lowLevelEngine.get()));
    else
      m_tracker.reset(new ITMColorTracker_CUDA(rgbImageSize, m_settings.noHierarchyLevels, m_settings.noRotationOnlyLevels, m_lowLevelEngine.get()));

    if(m_settings.useSwapping) m_swappingEngine.reset(new ITMSwappingEngine_CUDA<SpaintVoxel,ITMVoxelIndex>);

    m_visualisationEngine.reset(new ITMVisualisationEngine_CUDA<SpaintVoxel,ITMVoxelIndex>);
#else
    // This should never happen as things stand - we set useGPU to false if CUDA support isn't available.
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    // Use the CPU implementation of InfiniTAM.
    m_lowLevelEngine.reset(new ITMLowLevelEngine_CPU);
    m_sceneReconstructionEngine.reset(new ITMSceneReconstructionEngine_CPU<SpaintVoxel,ITMVoxelIndex>);

    if(m_settings.trackerType == ITMLibSettings::TRACKER_ICP)
      m_tracker.reset(new ITMDepthTracker_CPU(depthImageSize, m_settings.noHierarchyLevels, m_settings.noRotationOnlyLevels, m_settings.depthTrackerICPThreshold, m_lowLevelEngine.get()));
    else
      m_tracker.reset(new ITMColorTracker_CPU(rgbImageSize, m_settings.noHierarchyLevels, m_settings.noRotationOnlyLevels, m_lowLevelEngine.get()));

    if(m_settings.useSwapping) m_swappingEngine.reset(new ITMSwappingEngine_CPU<SpaintVoxel,ITMVoxelIndex>);

    m_visualisationEngine.reset(new ITMVisualisationEngine_CPU<SpaintVoxel,ITMVoxelIndex>);
  }

  // Note: The tracker can only be run once reconstruction has started.
  m_reconstructionStarted = false;
}

}
