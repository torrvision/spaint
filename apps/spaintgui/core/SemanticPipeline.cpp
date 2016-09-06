/**
 * spaintgui: SemanticPipeline.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "SemanticPipeline.h"
using namespace spaint;

//#################### CONSTRUCTORS ####################

SemanticPipeline::SemanticPipeline(const Settings_Ptr& settings, const std::string& resourcesDir, size_t maxLabelCount,
                                   const CompositeImageSourceEngine_Ptr& imageSourceEngine, unsigned int seed,
                                   TrackerType trackerType, const std::string& trackerParams,
                                   SLAMComponent::MappingMode mappingMode, SLAMComponent::TrackingMode trackingMode)
: MultiScenePipeline(settings, resourcesDir, maxLabelCount)
{
  const std::string sceneID = Model::get_world_scene_id();
  m_slamComponents[sceneID].reset(new SLAMComponent(m_model, sceneID, imageSourceEngine, trackerType, trackerParams, mappingMode, trackingMode));
  m_objectSegmentationComponents[sceneID].reset(new ObjectSegmentationComponent(m_model, sceneID));
  m_propagationComponents[sceneID].reset(new PropagationComponent(m_model, sceneID));
  m_semanticSegmentationComponents[sceneID].reset(new SemanticSegmentationComponent(m_model, sceneID, seed));
  m_smoothingComponents[sceneID].reset(new SmoothingComponent(m_model, sceneID));
}
