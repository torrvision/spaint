/**
 * spaint: SmoothingComponent.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SMOOTHINGCOMPONENT
#define H_SPAINT_SMOOTHINGCOMPONENT

#include <ITMLib/Objects/RenderStates/ITMRenderState.h>
#include <ITMLib/Utils/ITMLibSettings.h>

#include "SmoothingContext.h"
#include "../smoothing/interface/LabelSmoother.h"

namespace spaint {

/**
 * \brief TODO
 */
class SmoothingComponent
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;
  typedef boost::shared_ptr<const ITMLib::ITMLibSettings> Settings_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** TODO */
  SmoothingContext_Ptr m_context;

  /** The label smoother. */
  LabelSmoother_CPtr m_labelSmoother;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief TODO
   */
  explicit SmoothingComponent(const SmoothingContext_Ptr& context);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** TODO */
  virtual void run(const RenderState_CPtr& renderState);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<SmoothingComponent> SmoothingComponent_Ptr;

}

#endif
