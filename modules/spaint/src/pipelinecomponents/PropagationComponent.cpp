/**
 * spaint: PropagationComponent.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/PropagationComponent.h"

#include "propagation/LabelPropagatorFactory.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

PropagationComponent::PropagationComponent(const PropagationContext_Ptr& context, const std::string& sceneID)
: m_context(context), m_sceneID(sceneID)
{
  const Vector2i& depthImageSize = context->get_slam_state(sceneID)->get_depth_image_size();
  const int raycastResultSize = depthImageSize.width * depthImageSize.height;
  m_labelPropagator = LabelPropagatorFactory::make_label_propagator(raycastResultSize, context->get_settings(sceneID)->deviceType);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void PropagationComponent::run(const VoxelRenderState_CPtr& renderState)
{
  m_labelPropagator->propagate_label(m_context->get_semantic_label(), renderState->raycastResult, m_context->get_slam_state(m_sceneID)->get_voxel_scene().get());
}

}
