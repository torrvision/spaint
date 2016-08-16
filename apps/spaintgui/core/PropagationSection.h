/**
 * spaintgui: PropagationSection.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_PROPAGATIONSECTION
#define H_SPAINTGUI_PROPAGATIONSECTION

#include <ITMLib/Objects/RenderStates/ITMRenderState.h>
#include <ITMLib/Utils/ITMLibSettings.h>

#include <spaint/propagation/interface/LabelPropagator.h>

#include "PropagationState.h"

/**
 * \brief TODO
 */
class PropagationSection
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;
  typedef boost::shared_ptr<const ITMLib::ITMLibSettings> Settings_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The label propagator. */
  spaint::LabelPropagator_CPtr m_labelPropagator;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief TODO
   */
  PropagationSection(const Vector2i& depthImageSize, const Settings_CPtr& settings);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** TODO */
  virtual void run(PropagationState& state, const RenderState_CPtr& renderState);
};

#endif
