/**
 * spaintgui: TrainingSection.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_TRAININGSECTION
#define H_SPAINTGUI_TRAININGSECTION

#include <boost/shared_ptr.hpp>

#include <ITMLib/Objects/RenderStates/ITMRenderState.h>

#include "TrainingState.h"

/**
 * \brief TODO
 */
class TrainingSection
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** TODO */
  virtual void run(TrainingState& state, const RenderState_CPtr& samplingRenderState);
};

#endif
