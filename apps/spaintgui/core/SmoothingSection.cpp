/**
 * spaintgui: SmoothingSection.cpp
 */

#include "SmoothingSection.h"

//#################### PUBLIC MEMBER FUNCTIONS ####################

void SmoothingSection::run(SmoothingState& state, const RenderState_CPtr& renderState)
{
  state.get_label_smoother()->smooth_labels(renderState->raycastResult, state.get_model()->get_scene().get());
}
