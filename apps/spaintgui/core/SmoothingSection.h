/**
 * spaintgui: SmoothingSection.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_SMOOTHINGSECTION
#define H_SPAINTGUI_SMOOTHINGSECTION

#include <boost/shared_ptr.hpp>

#include <ITMLib/Objects/RenderStates/ITMRenderState.h>

#include "SmoothingState.h"

/**
 * \brief TODO
 */
class SmoothingSection
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** TODO */
  virtual void run(const SmoothingState& state, const RenderState_CPtr& renderState);
};

#endif
