/**
 * spaintgui: SmoothingSection.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "SmoothingSection.h"

//#################### PUBLIC MEMBER FUNCTIONS ####################

void SmoothingSection::run(SmoothingState& state, const RenderState_CPtr& renderState)
{
  state.get_label_smoother()->smooth_labels(renderState->raycastResult, state.get_model()->get_scene().get());
}
