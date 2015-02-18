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
SpaintPipeline::SpaintPipeline(const std::string& calibrationFilename, const boost::optional<std::string>& openNIDeviceURI, const Settings_Ptr& settings)
{
  m_imageSourceEngine.reset(new OpenNIEngine(calibrationFilename.c_str(), openNIDeviceURI ? openNIDeviceURI->c_str() : NULL));
  initialise(settings);
}

#ifdef WITH_VICON
SpaintPipeline::SpaintPipeline(const std::string& calibrationFilename, const boost::optional<std::string>& openNIDeviceURI, const Settings_CPtr& settings,
                               const std::string& viconHost)
: m_viconHost(viconHost)
{
  m_imageSourceEngine.reset(new OpenNIEngine(calibrationFilename.c_str(), openNIDeviceURI ? openNIDeviceURI->c_str() : NULL));
  initialise(settings);
}
#endif
#endif

SpaintPipeline::SpaintPipeline(const std::string& calibrationFilename, const std::string& rgbImageMask, const std::string& depthImageMask, const Settings_Ptr& settings)
{
  m_imageSourceEngine.reset(new ImageFileReader(calibrationFilename.c_str(), rgbImageMask.c_str(), depthImageMask.c_str()));
  initialise(settings);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

bool SpaintPipeline::get_fusion_enabled() const
{
  return m_fusionEnabled;
}

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
  const SpaintModel::View_Ptr& view = m_model->get_view();

  // Get the next frame.
  ITMView *newView = view.get();
  m_imageSourceEngine->getImages(m_inputRGBImage.get(), m_inputRawDepthImage.get());
  m_viewBuilder->UpdateView(&newView, m_inputRGBImage.get(), m_inputRawDepthImage.get());
  m_model->set_view(newView);

  // Track the camera (we can only do this once we've started reconstructing the model because we need something to track against).
  if(m_reconstructionStarted) m_trackingController->Track(trackingState.get(), view.get());

#ifdef WITH_VICON
  if(m_viconHost != "")
  {
    // If we're using the Vicon tracker, make sure to only fuse when we have tracking information available.
    m_fusionEnabled = !m_viconTracker->lost_tracking();
  }
#endif

  // Run the fusion process.
  if(m_fusionEnabled) m_denseMapper->ProcessFrame(view.get(), trackingState.get());

  // Raycast from the live camera position to prepare for tracking in the next frame.
  m_trackingController->Prepare(trackingState.get(), view.get());

  m_reconstructionStarted = true;
}

void SpaintPipeline::set_fusion_enabled(bool fusionEnabled)
{
  m_fusionEnabled = fusionEnabled;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void SpaintPipeline::initialise(const Settings_Ptr& settings)
{
  // Make sure that we're not trying to run on the GPU if CUDA support isn't enabled.
#ifndef WITH_CUDA
  if(settings->deviceType == ITMLibSettings::DEVICE_CUDA)
  {
    std::cerr << "[spaint] CUDA support unavailable, reverting to the CPU implementation of InfiniTAM\n";
    settings->deviceType = ITMLibSettings::DEVICE_CPU;
  }
#endif

  // Determine the RGB and depth image sizes.
  Vector2i rgbImageSize = m_imageSourceEngine->getRGBImageSize();
  Vector2i depthImageSize = m_imageSourceEngine->getDepthImageSize();
  if(depthImageSize.x == -1 || depthImageSize.y == -1) depthImageSize = rgbImageSize;

  // Set up the RGB and raw depth images into which input is to be read each frame.
  m_inputRGBImage.reset(new ITMUChar4Image(rgbImageSize, true, true));
  m_inputRawDepthImage.reset(new ITMShortImage(depthImageSize, true, true));

  // Set up the scene.
  MemoryDeviceType memoryType = settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
  SpaintModel::Scene_Ptr scene(new SpaintModel::Scene(&settings->sceneParams, settings->useSwapping, memoryType));

  // Set up the InfiniTAM engines and view builder.
  const ITMRGBDCalib *calib = &m_imageSourceEngine->calib;
  VisualisationEngine_Ptr visualisationEngine;
  if(settings->deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    // Use the CUDA implementations.
    m_lowLevelEngine.reset(new ITMLowLevelEngine_CUDA);
    m_viewBuilder.reset(new ITMViewBuilder_CUDA(calib));
    visualisationEngine.reset(new ITMVisualisationEngine_CUDA<SpaintVoxel,ITMVoxelIndex>(scene.get()));
#else
    // This should never happen as things stand - we set deviceType to DEVICE_CPU to false if CUDA support isn't available.
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    // Use the CPU implementations.
    m_lowLevelEngine.reset(new ITMLowLevelEngine_CPU);
    m_viewBuilder.reset(new ITMViewBuilder_CPU(calib));
    visualisationEngine.reset(new ITMVisualisationEngine_CPU<SpaintVoxel,ITMVoxelIndex>(scene.get()));
  }

  // Set up the live render state.
  Vector2i trackedImageSize = ITMTrackingController::GetTrackedImageSize(settings.get(), rgbImageSize, depthImageSize);
  RenderState_Ptr liveRenderState(visualisationEngine->CreateRenderState(trackedImageSize));

  // Set up the dense mapper and tracking controller.
  m_denseMapper.reset(new ITMDenseMapper<SpaintVoxel,ITMVoxelIndex>(settings.get(), scene.get(), liveRenderState.get()));
  setup_tracker(settings, scene, trackedImageSize);
  m_trackingController.reset(new ITMTrackingController(m_tracker.get(), visualisationEngine.get(), m_lowLevelEngine.get(), liveRenderState.get(), settings.get()));

  // Set up the spaint model and raycaster.
  TrackingState_Ptr trackingState(m_trackingController->BuildTrackingState());
  m_model.reset(new SpaintModel(scene, rgbImageSize, depthImageSize, trackingState, settings));
  m_raycaster.reset(new SpaintRaycaster(m_model, visualisationEngine, liveRenderState));

  m_fusionEnabled = true;
  m_reconstructionStarted = false;
}

void SpaintPipeline::setup_tracker(const Settings_CPtr& settings, const SpaintModel::Scene_Ptr& scene, const Vector2i& trackedImageSize)
{
#ifdef WITH_VICON
  if(m_viconHost != "")
  {
    ITMCompositeTracker *compositeTracker = new ITMCompositeTracker(2);
    m_viconTracker = new ViconTracker(m_viconHost, "kinect");
    compositeTracker->SetTracker(m_viconTracker, 0);
    compositeTracker->SetTracker(
      ITMTrackerFactory<SpaintVoxel,ITMVoxelIndex>::Instance().MakeICPTracker(
        trackedImageSize,
        settings.get(),
        m_lowLevelEngine.get(),
        m_imuCalibrator.get(),
        scene.get()
      ), 1
    );
    m_tracker.reset(compositeTracker);
  }
  else
  {
#endif
    m_imuCalibrator.reset(new ITMIMUCalibrator_iPad);
    m_tracker.reset(ITMTrackerFactory<SpaintVoxel,ITMVoxelIndex>::Instance().Make(
      trackedImageSize, settings.get(), m_lowLevelEngine.get(), m_imuCalibrator.get(), scene.get()
    ));
#ifdef WITH_VICON
  }
#endif
}

}
