/**
 * spaintgui: ObjectivePipeline.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "ObjectivePipeline.h"

#include <spaint/imagesources/SingleRGBDImagePipe.h>
using namespace spaint;

#include <tvgutil/containers/MapUtil.h>
#include <tvgutil/filesystem/PathFinder.h>
#include <tvgutil/timing/TimeUtil.h>
using namespace tvgutil;

//#################### CONSTRUCTORS ####################

ObjectivePipeline::ObjectivePipeline(const Settings_Ptr& settings, const std::string& resourcesDir, size_t maxLabelCount, const CompositeImageSourceEngine_Ptr& imageSourceEngine,
                                     TrackerType trackerType, const std::string& trackerParams, SLAMComponent::MappingMode mappingMode, SLAMComponent::TrackingMode trackingMode,
                                     const FiducialDetector_CPtr& fiducialDetector, bool detectFiducials, bool mirrorWorldPose)
: MultiScenePipeline(settings, resourcesDir, maxLabelCount)
{
  const std::string worldSceneID = Model::get_world_scene_id();
  SingleRGBDImagePipe_Ptr pipe(new SingleRGBDImagePipe(imageSourceEngine));
  m_slamComponents[worldSceneID].reset(new SLAMComponent(m_model, worldSceneID, imageSourceEngine, trackerType, trackerParams, mappingMode, trackingMode, fiducialDetector, detectFiducials));
  m_objectSegmentationComponents[worldSceneID].reset(new ObjectSegmentationComponent(m_model, worldSceneID, pipe));

  const std::string objectSceneID = "Object";
  SLAMComponent_Ptr objectSLAMComponent(new SLAMComponent(m_model, objectSceneID, pipe, trackerType, trackerParams, mappingMode, trackingMode));
  if(mirrorWorldPose) objectSLAMComponent->mirror_pose_of(worldSceneID);
  m_slamComponents[objectSceneID] = objectSLAMComponent;
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void ObjectivePipeline::set_mode(Mode mode)
{
  // If we are switching into segmentation training mode, reset the segmenter.
  if(mode == MODE_SEGMENTATION_TRAINING && m_mode != MODE_SEGMENTATION_TRAINING)
  {
    MapUtil::call_if_found(m_objectSegmentationComponents, Model::get_world_scene_id(), boost::bind(&ObjectSegmentationComponent::reset_segmenter, _1));
  }

  // If we are switching out of segmentation training mode, clear the segmentation image.
  if(m_mode == MODE_SEGMENTATION_TRAINING && mode != MODE_SEGMENTATION_TRAINING)
  {
    m_model->set_segmentation_image(Model::get_world_scene_id(), ITMUChar4Image_CPtr());
  }

  // If we are switching into segmentation mode, start a new segmentation video.
  boost::optional<SequentialPathGenerator>& segmentationPathGenerator = m_model->get_segmentation_path_generator();
  if(mode == MODE_SEGMENTATION && m_mode != MODE_SEGMENTATION)
  {
#if 0
    segmentationPathGenerator.reset(SequentialPathGenerator(find_subdir_from_executable("segmentations") / TimeUtil::get_iso_timestamp()));
    boost::filesystem::create_directories(segmentationPathGenerator->get_base_dir());
#endif
  }

  // If we are switching out of segmentation mode, stop recording the segmentation video
  // and clear the input mask and segmentation image.
  if(m_mode == MODE_SEGMENTATION && mode != MODE_SEGMENTATION)
  {
    segmentationPathGenerator.reset();
    m_model->get_slam_state(Model::get_world_scene_id())->set_input_mask(ITMUCharImage_Ptr());
    m_model->set_segmentation_image(Model::get_world_scene_id(), ITMUChar4Image_CPtr());
  }

  switch(mode)
  {
    case MODE_NORMAL:
    case MODE_SEGMENTATION:
    case MODE_SEGMENTATION_TRAINING:
      m_mode = mode;
      break;
    default:
      break;
  }
}
