/**
 * spaint: SmoothingComponent.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SMOOTHINGCOMPONENT
#define H_SPAINT_SMOOTHINGCOMPONENT

#include <ITMLib/Objects/RenderStates/ITMRenderState.h>

#include "SmoothingContext.h"
#include "../smoothing/interface/LabelSmoother.h"

namespace spaint {

/**
 * \brief An instance of this pipeline component can be used to smoothe the labels in a scene to remove noise.
 */
class SmoothingComponent
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The shared context needed for smoothing. */
  SmoothingContext_Ptr m_context;

  /** The label smoother. */
  LabelSmoother_CPtr m_labelSmoother;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a smoothing component.
   *
   * \param context The shared context needed for smoothing.
   */
  explicit SmoothingComponent(const SmoothingContext_Ptr& context);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Runs the smoothing component, smoothing the labels in the target scene to remove noise.
   *
   * \param renderState The render state associated with the camera position from which to smoothe.
   */
  void run(const RenderState_CPtr& renderState);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<SmoothingComponent> SmoothingComponent_Ptr;

}

#endif
