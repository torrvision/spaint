/**
 * spaint: SpaintPipeline.cpp
 */

#include "core/SpaintPipeline.h"

#ifdef WITH_OPENNI
#include <Engine/OpenNIEngine.h>
#endif
#include <ITMLib/Engine/ITMRenTracker.cpp>
#include <ITMLib/Engine/ITMTrackerFactory.h>
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
  m_imageSourceEngine->getImages(view.get());

  // If we're using the GPU, transfer the relevant images in the frame across to it.
  if(m_model->get_settings().useGPU)
  {
    view->rgb->UpdateDeviceFromHost();

    switch(view->inputImageType)
    {
      case ITMView::InfiniTAM_FLOAT_DEPTH_IMAGE:
        view->depth->UpdateDeviceFromHost();
        break;
      case ITMView::InfiniTAM_SHORT_DEPTH_IMAGE:
      case ITMView::InfiniTAM_DISPARITY_IMAGE:
        view->rawDepth->UpdateDeviceFromHost();
        break;
      default:
        // This should never happen.
        throw std::runtime_error("Error: SpaintPipeline::process_frame: Unknown input image type");
    }
  }

  // If the frame only contains a short depth image or a disparity image, make a proper floating-point depth image from what we have.
  // Note that this will automatically run on either the GPU or the CPU behind the scenes, depending on which mode we're in.
  switch(view->inputImageType)
  {
    case ITMView::InfiniTAM_SHORT_DEPTH_IMAGE:
      m_lowLevelEngine->ConvertDepthMMToFloat(view->depth, view->rawDepth);
      break;
    case ITMView::InfiniTAM_DISPARITY_IMAGE:
      m_lowLevelEngine->ConvertDisparityToDepth(view->depth, view->rawDepth, &view->calib->intrinsics_d, &view->calib->disparityCalib);
      break;
    default:
      break;
  }

  // Track the camera (we can only do this once we've started reconstructing the model because we need something to track against).
  if(m_reconstructionStarted)
  {
    if(m_trackerPrimary) m_trackerPrimary->TrackCamera(trackingState.get(), view.get());
    if(m_trackerSecondary) m_trackerSecondary->TrackCamera(trackingState.get(), view.get());
  }

  // Allocate voxel blocks as necessary.
  m_sceneReconstructionEngine->AllocateSceneFromDepth(scene.get(), view.get(), trackingState->pose_d);

  // Integrate (fuse) the view into the scene.
  m_sceneReconstructionEngine->IntegrateIntoScene(scene.get(), view.get(), trackingState->pose_d);

  // Swap voxel blocks between the GPU and CPU.
  if(m_model->get_settings().useSwapping)
  {
    // CPU -> GPU
    m_swappingEngine->IntegrateGlobalIntoLocal(scene.get(), view.get());

    // GPU -> CPU
    m_swappingEngine->SaveToGlobalMemory(scene.get(), view.get());
  }

  // Perform raycasting to visualise the scene.
  // FIXME: It would be better to use dynamic dispatch for this.
  const SpaintRaycaster::VisualisationEngine_Ptr& visualisationEngine = m_raycaster->get_visualisation_engine();
  switch(m_model->get_settings().trackerType)
  {
    case ITMLibSettings::TRACKER_ICP:
    case ITMLibSettings::TRACKER_REN:
    {
      visualisationEngine->CreateExpectedDepths(scene.get(), trackingState->pose_d, &view->calib->intrinsics_d, trackingState->renderingRangeImage);
      visualisationEngine->CreateICPMaps(scene.get(), view.get(), trackingState.get());
      break;
    }
    case ITMLibSettings::TRACKER_COLOR:
    {
      ITMPose rgbPose(view->calib->trafo_rgb_to_depth.calib_inv * trackingState->pose_d->M);
      visualisationEngine->CreateExpectedDepths(scene.get(), &rgbPose, &view->calib->intrinsics_rgb, trackingState->renderingRangeImage);
      visualisationEngine->CreatePointCloud(scene.get(), view.get(), trackingState.get(), m_model->get_settings().skipPoints);
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
  if(settings.useGPU)
  {
    std::cerr << "[spaint] CUDA support unavailable, reverting to the CPU implementation of InfiniTAM\n";
    settings.useGPU = false;
  }
#endif

  // Determine the RGB and depth image sizes.
  Vector2i rgbImageSize = m_imageSourceEngine->getRGBImageSize();
  Vector2i depthImageSize = m_imageSourceEngine->getDepthImageSize();
  if(depthImageSize.x == -1 || depthImageSize.y == -1) depthImageSize = rgbImageSize;

  // Set up the spaint model.
  m_model.reset(new SpaintModel(settings, rgbImageSize, depthImageSize, m_imageSourceEngine->calib));

  // Set up the InfiniTAM engines.
  if(settings.deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    // Use the GPU implementation of InfiniTAM.
    m_lowLevelEngine.reset(new ITMLowLevelEngine_CUDA);
    m_sceneReconstructionEngine.reset(new ITMSceneReconstructionEngine_CUDA<SpaintVoxel,ITMVoxelIndex>);
    if(settings.useSwapping) m_swappingEngine.reset(new ITMSwappingEngine_CUDA<SpaintVoxel,ITMVoxelIndex>);
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
    if(settings.useSwapping) m_swappingEngine.reset(new ITMSwappingEngine_CPU<SpaintVoxel,ITMVoxelIndex>);
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
