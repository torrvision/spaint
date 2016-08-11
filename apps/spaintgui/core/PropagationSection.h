/**
 * spaintgui: PropagationSection.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_PROPAGATIONSECTION
#define H_SPAINTGUI_PROPAGATIONSECTION

#include <boost/shared_ptr.hpp>

#include <ITMLib/Objects/RenderStates/ITMRenderState.h>

#include "PropagationState.h"

/**
 * \brief TODO
 */
class PropagationSection
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** TODO */
  virtual void run(PropagationState& state, const RenderState_CPtr& renderState);
};

#endif
