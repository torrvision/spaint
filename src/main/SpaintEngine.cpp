/**
 * spaint: SpaintEngine.cpp
 */

#include "main/SpaintEngine.h"

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
