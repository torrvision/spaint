/**
 * spaintgui: PropagationSection.cpp
 */

#include "PropagationSection.h"

//#################### PUBLIC MEMBER FUNCTIONS ####################

void PropagationSection::run(PropagationState& state, const RenderState_CPtr& renderState)
{
  state.get_label_propagator()->propagate_label(state.get_interactor()->get_semantic_label(), renderState->raycastResult, state.get_model()->get_scene().get());
}
