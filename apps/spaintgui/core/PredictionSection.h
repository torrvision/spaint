/**
 * spaintgui: PredictionSection.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_PREDICTIONSECTION
#define H_SPAINTGUI_PREDICTIONSECTION

#include <boost/shared_ptr.hpp>

#include <ITMLib/Objects/RenderStates/ITMRenderState.h>

#include "PredictionState.h"

/**
 * \brief TODO
 */
class PredictionSection
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** TODO */
  virtual void run(PredictionState& state, const RenderState_CPtr& samplingRenderState);
};

#endif
