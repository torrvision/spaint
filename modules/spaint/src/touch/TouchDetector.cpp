/**
 * spaint: TouchDetector.cpp
 */

#include "touch/TouchDetector.h"

namespace spaint {

//#################### CONSTRUCTORS #################### 
TouchDetector::TouchDetector(){}

//#################### PUBLIC MEMBER FUNCTIONS #################### 
void TouchDetector::run_touch_detector_on_frame(const RenderState_Ptr& renderState, const rigging::SimpleCamera_Ptr camera, float voxelSize, ITMFloatImage *rawDepth) const
{
}

const TouchState& TouchDetector::get_touch_state() const
{
  return m_touchState;
}

//#################### PRIVATE MEMBER FUNCTIONS #################### 
void TouchDetector::generate_depth_raycast(const FloatImage_Ptr& output, const RenderState_Ptr& renderState, const rigging::SimpleCamera_Ptr camera, float voxelSize) const
{
}
}

