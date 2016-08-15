/**
 * spaintgui: SmoothingSection.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_SMOOTHINGSECTION
#define H_SPAINTGUI_SMOOTHINGSECTION

#include <ITMLib/Objects/RenderStates/ITMRenderState.h>

#include <spaint/smoothing/interface/LabelSmoother.h>

#include "SmoothingState.h"

/**
 * \brief TODO
 */
class SmoothingSection
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;
  typedef boost::shared_ptr<const ITMLib::ITMLibSettings> Settings_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The label smoother. */
  spaint::LabelSmoother_CPtr m_labelSmoother;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief TODO
   */
  SmoothingSection(size_t maxLabelCount, const Settings_CPtr& settings);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** TODO */
  virtual void run(SmoothingState& state, const RenderState_CPtr& renderState);
};

#endif
