/**
 * spaint: PropagationComponent.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/PropagationComponent.h"

#include "propagation/LabelPropagatorFactory.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

PropagationComponent::PropagationComponent(const Vector2i& depthImageSize, const Settings_CPtr& settings)
{
  const int raycastResultSize = depthImageSize.width * depthImageSize.height;
  m_labelPropagator = LabelPropagatorFactory::make_label_propagator(raycastResultSize, settings->deviceType);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void PropagationComponent::run(PropagationModel& model, const RenderState_CPtr& renderState)
{
  m_labelPropagator->propagate_label(model.get_semantic_label(), renderState->raycastResult, model.get_scene().get());
}

}
