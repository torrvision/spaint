/**
 * spaint: SpaintEngine.cpp
 */

#include "SpaintEngine.h"

#include <stdexcept>

#include <Engine/ImageSourceEngine.h>
#ifdef WITH_OPENNI
#include <Engine/OpenNIEngine.h>
#endif
#include <ITMLib/Engine/ITMRenTracker.cpp>
#include <ITMLib/Engine/ITMTrackerFactory.h>
#include <ITMLib/Engine/ITMVisualisationEngine.cpp>
#include <ITMLib/Engine/DeviceSpecific/CPU/ITMRenTracker_CPU.cpp>
#include <ITMLib/Engine/DeviceSpecific/CPU/ITMSceneReconstructionEngine_CPU.cpp>
#include <ITMLib/Engine/DeviceSpecific/CPU/ITMSwappingEngine_CPU.cpp>
#include <ITMLib/Engine/DeviceSpecific/CPU/ITMVisualisationEngine_CPU.cpp>
using namespace InfiniTAM::Engine;

#include "trackers/ViconTracker.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

#ifdef WITH_OPENNI
SpaintEngine::SpaintEngine(const std::string& calibrationFilename, const boost::shared_ptr<std::string>& openNIDeviceURI, const ITMLibSettings& settings, bool useVicon)
: m_settings(settings), m_useVicon(useVicon)
{
  m_imageSourceEngine.reset(new OpenNIEngine(calibrationFilename.c_str(), openNIDeviceURI ? openNIDeviceURI->c_str() : NULL));
  initialise();
}
#endif

SpaintEngine::SpaintEngine(const std::string& calibrationFilename, const std::string& rgbImageMask, const std::string& depthImageMask, const ITMLibSettings& settings)
: m_settings(settings), m_useVicon(false)
{
  m_imageSourceEngine.reset(new ImageFileReader(calibrationFilename.c_str(), rgbImageMask.c_str(), depthImageMask.c_str()));
  initialise();
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void SpaintEngine::generate_free_raycast(const UChar4Image_Ptr& output, const ITMPose& pose) const
{
  if(!m_visualisationState) m_visualisationState.reset(m_visualisationEngine->allocateInternalState(output->noDims));

  m_visualisationEngine->FindVisibleBlocks(m_scene.get(), &pose, &m_view->calib->intrinsics_d, m_visualisationState.get());
  m_visualisationEngine->CreateExpectedDepths(m_scene.get(), &pose, &m_view->calib->intrinsics_d, m_visualisationState->minmaxImage, m_visualisationState.get());
  m_visualisationEngine->RenderImage(m_scene.get(), &pose, &m_view->calib->intrinsics_d, m_visualisationState.get(), m_visualisationState->outputImage, false);

  if(m_settings.useGPU) m_visualisationState->outputImage->UpdateHostFromDevice();
  output->SetFrom(m_visualisationState->outputImage);
}

void SpaintEngine::get_default_raycast(const UChar4Image_Ptr& output) const
{
  prepare_to_copy_visualisation(m_trackingState->rendering, output);
  output->SetFrom(m_trackingState->rendering);
}

void SpaintEngine::get_depth_input(const UChar4Image_Ptr& output) const
{
  prepare_to_copy_visualisation(m_view->depth, output);
  m_visualisationEngine->DepthToUchar4(output.get(), m_view->depth);
}

SpaintEngine::ImageSourceEngine_Ptr SpaintEngine::get_image_source_engine() const
{
  return m_imageSourceEngine;
}

const ITMIntrinsics& SpaintEngine::get_intrinsics() const
{
  return m_view->calib->intrinsics_d;
}

const ITMPose& SpaintEngine::get_pose() const
{
  return *m_trackingState->pose_d;
}

void SpaintEngine::get_rgb_input(const UChar4Image_Ptr& output) const
{
  prepare_to_copy_visualisation(m_view->rgb, output);
  output->SetFrom(m_view->rgb);
}

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
  if(m_reconstructionStarted)
  {
    if(m_trackerPrimary) m_trackerPrimary->TrackCamera(m_trackingState.get(), m_view.get());
    if(m_trackerSecondary) m_trackerSecondary->TrackCamera(m_trackingState.get(), m_view.get());
    std::cout << m_trackingState->pose_d->M << '\n';
  }

  // Allocate voxel blocks as necessary.
  m_sceneReconstructionEngine->AllocateSceneFromDepth(m_scene.get(), m_view.get(), m_trackingState->pose_d);

  // Integrate (fuse) the view into the scene.
  m_sceneReconstructionEngine->IntegrateIntoScene(m_scene.get(), m_view.get(), m_trackingState->pose_d);

  // Swap voxel blocks between the GPU and CPU.
  if(m_settings.useSwapping)
  {
    // CPU -> GPU
    m_swappingEngine->IntegrateGlobalIntoLocal(m_scene.get(), m_view.get());

    // GPU -> CPU
    m_swappingEngine->SaveToGlobalMemory(m_scene.get(), m_view.get());
  }

  // Perform raycasting to visualise the scene.
  // FIXME: It would be better to use dynamic dispatch for this.
  switch(m_settings.trackerType)
  {
    case ITMLibSettings::TRACKER_ICP:
    case ITMLibSettings::TRACKER_REN:
    {
      m_visualisationEngine->CreateExpectedDepths(m_scene.get(), m_trackingState->pose_d, &m_view->calib->intrinsics_d, m_trackingState->renderingRangeImage);
      m_visualisationEngine->CreateICPMaps(m_scene.get(), m_view.get(), m_trackingState.get());
      break;
    }
    case ITMLibSettings::TRACKER_COLOR:
    {
      ITMPose rgbPose(m_view->calib->trafo_rgb_to_depth.calib_inv * m_trackingState->pose_d->M);
      m_visualisationEngine->CreateExpectedDepths(m_scene.get(), &rgbPose, &m_view->calib->intrinsics_rgb, m_trackingState->renderingRangeImage);
      m_visualisationEngine->CreatePointCloud(m_scene.get(), m_view.get(), m_trackingState.get(), m_settings.skipPoints);
      break;
    }
    default:
    {
      throw std::runtime_error("Error: SpaintEngine::process_frame: Unknown tracker type");
    }
  }

  m_reconstructionStarted = true;
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
  m_trackingState.reset(ITMTrackerFactory::MakeTrackingState(m_settings, rgbImageSize, depthImageSize));
  m_trackingState->pose_d->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

  // Set up the scene view.
  m_view.reset(new ITMView(m_imageSourceEngine->calib, rgbImageSize, depthImageSize, m_settings.useGPU));

  // Set up the InfiniTAM engines.
  if(m_settings.useGPU)
  {
#ifdef WITH_CUDA
    // Use the GPU implementation of InfiniTAM.
    m_lowLevelEngine.reset(new ITMLowLevelEngine_CUDA);
    m_sceneReconstructionEngine.reset(new ITMSceneReconstructionEngine_CUDA<SpaintVoxel,ITMVoxelIndex>);
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
    if(m_settings.useSwapping) m_swappingEngine.reset(new ITMSwappingEngine_CPU<SpaintVoxel,ITMVoxelIndex>);
    m_visualisationEngine.reset(new ITMVisualisationEngine_CPU<SpaintVoxel,ITMVoxelIndex>);
  }

  // Set up the trackers.
  if(m_useVicon)
  {
    // Note: Need to enable port forwarding to make this work.
    m_trackerPrimary.reset(new ViconTracker("192.168.0.111", "kinect"));
    //m_trackerPrimary.reset(ITMTrackerFactory::MakePrimaryTracker(m_settings, rgbImageSize, depthImageSize, m_lowLevelEngine.get()));
    //m_trackerSecondary.reset(new ViconTracker("192.168.0.111", "kinect"));
  }
  else
  {
    m_trackerPrimary.reset(ITMTrackerFactory::MakePrimaryTracker(m_settings, rgbImageSize, depthImageSize, m_lowLevelEngine.get()));
    m_trackerSecondary.reset(ITMTrackerFactory::MakeSecondaryTracker<SpaintVoxel,ITMVoxelIndex>(m_settings, rgbImageSize, depthImageSize, m_lowLevelEngine.get(), m_scene.get()));
  }

  // Note: The trackers can only be run once reconstruction has started.
  m_reconstructionStarted = false;
}

}
