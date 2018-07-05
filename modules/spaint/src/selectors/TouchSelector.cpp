/*
 * spaint: TouchSelector.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "selectors/TouchSelector.h"

#include <boost/serialization/extended_type_info.hpp>
#include <boost/serialization/singleton.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include <itmx/picking/cpu/Picker_CPU.h>
#ifdef WITH_CUDA
#include <itmx/picking/cuda/Picker_CUDA.h>
#endif
#include <itmx/util/CameraPoseConverter.h>
using namespace itmx;

#include <tvgutil/timing/Timer.h>

using namespace ITMLib;
using namespace rigging;
using namespace tvginput;

#define DEBUGGING 0

namespace spaint {

//#################### CONSTRUCTORS ####################

TouchSelector::TouchSelector(const Settings_CPtr& itmSettings, const TouchSettings_Ptr& touchSettings, const Vector2i& touchImageSize, size_t maxKeptTouchPoints)
: Selector(itmSettings),
  m_keptTouchPointCount(0),  
  m_keptTouchPointsFloatMB(new ORUtils::MemoryBlock<Vector3f>(maxKeptTouchPoints, true, true)),
  m_keptTouchPointsShortMB(new ORUtils::MemoryBlock<Vector3s>(maxKeptTouchPoints, true, true)),
  m_maxKeptTouchPoints(maxKeptTouchPoints),
  m_touchDetector(new TouchDetector(touchImageSize, itmSettings, touchSettings))
{
  m_isActive = true;

  // Make the picker.
  if(m_settings->deviceType == DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    m_picker.reset(new Picker_CUDA);
#else
    // This should never happen as things stand - we set deviceType to DEVICE_CPU if CUDA support isn't available.
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    m_picker.reset(new Picker_CPU);
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void TouchSelector::accept(const SelectorVisitor& visitor) const
{
  visitor.visit(*this);
}

ITMUChar4Image_CPtr TouchSelector::generate_touch_image(const View_CPtr& view) const
{
  return m_touchDetector->generate_touch_image(view);
}

std::vector<Eigen::Vector3f> TouchSelector::get_positions() const
{
  // If the last update did not yield any valid touch points, early out.
  if(m_keptTouchPointCount == 0) return std::vector<Eigen::Vector3f>();

  // Convert the touch points from voxel coordinates into scene coordinates and return them.
  return Picker::get_positions<Eigen::Vector3f>(*m_keptTouchPointsFloatMB, m_settings->sceneParams.voxelSize);
}

Selector::Selection_CPtr TouchSelector::get_selection() const
{
  return m_keptTouchPointCount > 0 ? m_keptTouchPointsShortMB : Selection_CPtr();
}

void TouchSelector::update(const InputState& inputState, const SLAMState_CPtr& slamState, const VoxelRenderState_CPtr& renderState, bool renderingInMono)
{
  // Detect any points that the user is touching in the scene.
  MoveableCamera_CPtr camera(new SimpleCamera(CameraPoseConverter::pose_to_camera(slamState->get_pose())));
  ITMFloatImage_Ptr depthImage(slamState->get_view()->depth, boost::serialization::null_deleter());
  TIME(std::vector<Eigen::Vector2i> touchPoints = m_touchDetector->determine_touch_points(camera, depthImage, renderState), milliseconds, runningTouchDetectorOnFrame);
#if DEBUGGING
  std::cout << runningTouchDetectorOnFrame << '\n';
#endif

  // Determine which of the touch points are valid (i.e. are ones that we want to keep) and copy them into a memory block.
  // Note that we limit the overall number of points we keep for performance reasons, so not all of the valid touch points
  // may end up being retained.
  m_keptTouchPointsFloatMB->Clear();
  m_keptTouchPointCount = 0;
  for(size_t i = 0, touchPointCount = touchPoints.size(); i < touchPointCount && m_keptTouchPointCount < m_maxKeptTouchPoints; ++i)
  {
    if(m_picker->pick(touchPoints[i][0], touchPoints[i][1], renderState.get(), *m_keptTouchPointsFloatMB, m_keptTouchPointCount))
    {
      ++m_keptTouchPointCount;
    }
  }

  // Convert the touch points we kept to Vector3s format.
  if(m_keptTouchPointCount > 0)
  {
    m_picker->to_short(*m_keptTouchPointsFloatMB, *m_keptTouchPointsShortMB);
  }
}

}
