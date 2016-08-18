/**
 * spaint: FeatureInspectionComponent.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_FEATUREINSPECTIONCOMPONENT
#define H_SPAINT_FEATUREINSPECTIONCOMPONENT

#include <ITMLib/Objects/RenderStates/ITMRenderState.h>

#include "FeatureInspectionModel.h"

namespace spaint {

/**
 * \brief TODO
 */
class FeatureInspectionComponent
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** TODO */
  virtual void run(FeatureInspectionModel& model, const RenderState_CPtr& renderState);
};

}

#endif
