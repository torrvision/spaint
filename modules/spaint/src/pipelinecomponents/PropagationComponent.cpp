/**
 * spaint: PropagationComponent.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/PropagationComponent.h"

#include "propagation/LabelPropagatorFactory.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

PropagationComponent::PropagationComponent(const PropagationContext_Ptr& context)
: m_context(context)
{
  const Vector2i& depthImageSize = context->get_depth_image_size();
  const int raycastResultSize = depthImageSize.width * depthImageSize.height;
  m_labelPropagator = LabelPropagatorFactory::make_label_propagator(raycastResultSize, context->get_settings()->deviceType);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void PropagationComponent::run(const RenderState_CPtr& renderState)
{
  m_labelPropagator->propagate_label(m_context->get_semantic_label(), renderState->raycastResult, m_context->get_scene().get());
}

}
