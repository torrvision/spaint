/**
 * spaintgui: SLAMSection.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_SLAMSECTION
#define H_SPAINTGUI_SLAMSECTION

#include <boost/shared_ptr.hpp>

#include <ITMLib/Objects/RenderStates/ITMRenderState.h>

#include "SLAMState.h"

/**
 * \brief TODO
 */
class SLAMSection
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The number of frames for which fusion has been run. */
  size_t m_fusedFramesCount;

  /**
   * A number of initial frames to fuse, regardless of their tracking quality.
   * Tracking quality can be poor in the first few frames, when there is only
   * a limited model against which to track. By forcibly fusing these frames,
   * we prevent poor tracking quality from stopping the reconstruction. After
   * these frames have been fused, only frames with a good tracking result will
   * be fused.
   */
  size_t m_initialFramesToFuse;

  /** The remaining number of frames for which we need to achieve good tracking before we can add another keyframe. */
  size_t m_keyframeDelay;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief TODO
   */
  SLAMSection();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** TODO */
  virtual void run(SLAMState& state);
};

#endif
