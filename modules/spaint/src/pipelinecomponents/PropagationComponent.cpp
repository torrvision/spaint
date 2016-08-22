/**
 * spaint: PropagationComponent.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/PropagationComponent.h"

#include "propagation/LabelPropagatorFactory.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

PropagationComponent::PropagationComponent(const PropagationContext_Ptr& context, const Vector2i& depthImageSize, const Settings_CPtr& settings)
: m_context(context)
{
  const int raycastResultSize = depthImageSize.width * depthImageSize.height;
  m_labelPropagator = LabelPropagatorFactory::make_label_propagator(raycastResultSize, settings->deviceType);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void PropagationComponent::run(const RenderState_CPtr& renderState)
{
  m_labelPropagator->propagate_label(m_context->get_semantic_label(), renderState->raycastResult, m_context->get_scene().get());
}

}
