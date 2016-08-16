/**
 * spaintgui: PropagationSection.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "PropagationSection.h"

#include <spaint/propagation/LabelPropagatorFactory.h>
using namespace spaint;

//#################### CONSTRUCTORS ####################

PropagationSection::PropagationSection(const Vector2i& depthImageSize, const Settings_CPtr& settings)
{
  const int raycastResultSize = depthImageSize.width * depthImageSize.height;
  m_labelPropagator = LabelPropagatorFactory::make_label_propagator(raycastResultSize, settings->deviceType);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void PropagationSection::run(PropagationState& state, const RenderState_CPtr& renderState)
{
  m_labelPropagator->propagate_label(state.get_semantic_label(), renderState->raycastResult, state.get_scene().get());
}
