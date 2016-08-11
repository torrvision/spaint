/**
 * spaintgui: FeatureInspectionSection.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_FEATUREINSPECTIONSECTION
#define H_SPAINTGUI_FEATUREINSPECTIONSECTION

#include <boost/shared_ptr.hpp>

#include <ITMLib/Objects/RenderStates/ITMRenderState.h>

#include "FeatureInspectionState.h"

/**
 * \brief TODO
 */
class FeatureInspectionSection
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** TODO */
  virtual void run(FeatureInspectionState& state, const RenderState_CPtr& renderState);
};

#endif
