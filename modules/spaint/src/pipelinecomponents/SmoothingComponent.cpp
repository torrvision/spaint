/**
 * spaint: SmoothingComponent.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/SmoothingComponent.h"

#include "smoothing/LabelSmootherFactory.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

SmoothingComponent::SmoothingComponent(const SmoothingContext_Ptr& context, const std::string& sceneID)
: m_context(context), m_sceneID(sceneID)
{
  size_t maxLabelCount = context->get_label_manager()->get_max_label_count();
  m_labelSmoother = LabelSmootherFactory::make_label_smoother(maxLabelCount, context->get_settings(sceneID)->deviceType);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void SmoothingComponent::run(const VoxelRenderState_CPtr& renderState)
{
  m_labelSmoother->smooth_labels(renderState->raycastResult, m_context->get_slam_state(m_sceneID)->get_voxel_scene().get());
}

}
