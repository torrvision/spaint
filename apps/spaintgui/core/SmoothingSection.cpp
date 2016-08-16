/**
 * spaintgui: SmoothingSection.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "SmoothingSection.h"

#include <spaint/smoothing/LabelSmootherFactory.h>
using namespace spaint;

//#################### CONSTRUCTORS ####################

SmoothingSection::SmoothingSection(size_t maxLabelCount, const Settings_CPtr& settings)
{
  m_labelSmoother = LabelSmootherFactory::make_label_smoother(maxLabelCount, settings->deviceType);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void SmoothingSection::run(SmoothingState& state, const RenderState_CPtr& renderState)
{
  m_labelSmoother->smooth_labels(renderState->raycastResult, state.get_scene().get());
}
