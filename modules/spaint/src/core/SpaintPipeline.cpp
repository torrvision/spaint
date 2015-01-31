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
SpaintPipeline::SpaintPipeline(const std::string& calibrationFilename, const boost::shared_ptr<std::string>& openNIDeviceURI, const ITMLibSettings& settings)
{
  m_imageSourceEngine.reset(new OpenNIEngine(calibrationFilename.c_str(), openNIDeviceURI ? openNIDeviceURI->c_str() : NULL));
  initialise(settings);
}
#endif

SpaintPipeline::SpaintPipeline(const std::string& calibrationFilename, const std::string& rgbImageMask, const std::string& depthImageMask, const ITMLibSettings& settings)
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

  const SpaintModel::Scene_Ptr& scene = m_model->get_scene();
  const SpaintModel::TrackingState_Ptr& trackingState = m_model->get_tracking_state();
  const SpaintModel::View_Ptr& view = m_model->get_view();

  // Get the next frame.
  m_imageSourceEngine->getImages(m_inputRGBImage.get(), m_inputRawDepthImage.get());
  m_viewBuilder->UpdateView(m_model->get_view().get(), m_inputRGBImage.get(), m_inputRawDepthImage.get());

  // Track the camera (we can only do this once we've started reconstructing the model because we need something to track against).
  if(m_reconstructionStarted)
  {
    if(m_trackerPrimary) m_trackerPrimary->TrackCamera(trackingState.get(), view.get());
    if(m_trackerSecondary) m_trackerSecondary->TrackCamera(trackingState.get(), view.get());
  }

  // Allocate voxel blocks as necessary.
  RenderState_Ptr liveRenderState = m_raycaster->get_live_render_state();
  m_sceneReconstructionEngine->AllocateSceneFromDepth(scene.get(), view.get(), trackingState.get(), liveRenderState.get());

  // Integrate (fuse) the view into the scene.
  m_sceneReconstructionEngine->IntegrateIntoScene(scene.get(), view.get(), trackingState.get(), liveRenderState.get());

  // Swap voxel blocks between the GPU and CPU.
  if(m_model->get_settings().useSwapping)
  {
    // CPU -> GPU
    m_swappingEngine->IntegrateGlobalIntoLocal(scene.get(), liveRenderState.get());

    // GPU -> CPU
    m_swappingEngine->SaveToGlobalMemory(scene.get(), liveRenderState.get());
  }

  // Perform raycasting to visualise the scene.
  // FIXME: It would be better to use dynamic dispatch for this.
  const SpaintRaycaster::VisualisationEngine_Ptr& visualisationEngine = m_raycaster->get_visualisation_engine();
  switch(m_model->get_settings().trackerType)
  {
    case ITMLibSettings::TRACKER_ICP:
    case ITMLibSettings::TRACKER_REN:
    {
      visualisationEngine->CreateExpectedDepths(trackingState->pose_d, &view->calib->intrinsics_d, liveRenderState.get());
      visualisationEngine->CreateICPMaps(view.get(), trackingState.get(), liveRenderState.get());
      break;
    }
    case ITMLibSettings::TRACKER_COLOR:
    {
      ITMPose rgbPose(view->calib->trafo_rgb_to_depth.calib_inv * trackingState->pose_d->GetM());
      visualisationEngine->CreateExpectedDepths(&rgbPose, &view->calib->intrinsics_rgb, liveRenderState.get());
      visualisationEngine->CreatePointCloud(view.get(), trackingState.get(), liveRenderState.get(), m_model->get_settings().skipPoints);
      break;
    }
    default:
    {
      throw std::runtime_error("Error: SpaintPipeline::process_frame: Unknown tracker type");
    }
  }

  m_reconstructionStarted = true;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void SpaintPipeline::initialise(ITMLibSettings settings)
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
  m_model.reset(new SpaintModel(settings, rgbImageSize, depthImageSize));

  // Set up the InfiniTAM engines and view builder.
  const ITMRGBDCalib *calib = &m_imageSourceEngine->calib;
  if(settings.deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    // Use the GPU implementation of InfiniTAM.
    m_lowLevelEngine.reset(new ITMLowLevelEngine_CUDA);
    m_sceneReconstructionEngine.reset(new ITMSceneReconstructionEngine_CUDA<SpaintVoxel,ITMVoxelIndex>);
    if(settings.useSwapping) m_swappingEngine.reset(new ITMSwappingEngine_CUDA<SpaintVoxel,ITMVoxelIndex>);
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
    m_sceneReconstructionEngine.reset(new ITMSceneReconstructionEngine_CPU<SpaintVoxel,ITMVoxelIndex>);
    if(settings.useSwapping) m_swappingEngine.reset(new ITMSwappingEngine_CPU<SpaintVoxel,ITMVoxelIndex>);
    m_viewBuilder.reset(new ITMViewBuilder_CPU(calib));
  }

  // Set up the raycaster.
  m_raycaster.reset(new SpaintRaycaster(m_model));

  // Set up the trackers.
  m_trackerPrimary.reset(ITMTrackerFactory::MakePrimaryTracker(settings, rgbImageSize, depthImageSize, m_lowLevelEngine.get()));
  m_trackerSecondary.reset(ITMTrackerFactory::MakeSecondaryTracker<SpaintVoxel,ITMVoxelIndex>(settings, rgbImageSize, depthImageSize, m_lowLevelEngine.get(), m_model->get_scene().get()));

  // Note: The trackers can only be run once reconstruction has started.
  m_reconstructionStarted = false;
}

}
