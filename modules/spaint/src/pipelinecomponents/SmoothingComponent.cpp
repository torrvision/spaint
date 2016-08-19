/**
 * spaint: SmoothingComponent.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/SmoothingComponent.h"

#include "smoothing/LabelSmootherFactory.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

SmoothingComponent::SmoothingComponent(size_t maxLabelCount, const Settings_CPtr& settings)
{
  m_labelSmoother = LabelSmootherFactory::make_label_smoother(maxLabelCount, settings->deviceType);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void SmoothingComponent::run(SmoothingContext& context, const RenderState_CPtr& renderState)
{
  m_labelSmoother->smooth_labels(renderState->raycastResult, context.get_scene().get());
}

}
