/**
 * spaint: PropagationComponent.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_PROPAGATIONCOMPONENT
#define H_SPAINT_PROPAGATIONCOMPONENT

#include <ITMLib/Objects/RenderStates/ITMRenderState.h>
#include <ITMLib/Utils/ITMLibSettings.h>

#include "PropagationModel.h"
#include "../propagation/interface/LabelPropagator.h"

namespace spaint {

/**
 * \brief TODO
 */
class PropagationComponent
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;
  typedef boost::shared_ptr<const ITMLib::ITMLibSettings> Settings_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The label propagator. */
  LabelPropagator_CPtr m_labelPropagator;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief TODO
   */
  PropagationComponent(const Vector2i& depthImageSize, const Settings_CPtr& settings);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** TODO */
  virtual void run(PropagationModel& model, const RenderState_CPtr& renderState);
};

}

#endif
