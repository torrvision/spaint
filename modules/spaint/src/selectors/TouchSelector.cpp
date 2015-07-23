/*
 * spaint: TouchSelector.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "selectors/TouchSelector.h"

#include <boost/serialization/extended_type_info.hpp>
#include <boost/serialization/singleton.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include <tvgutil/timing/Timer.h>

#include "picking/cpu/Picker_CPU.h"
#include "util/CameraPoseConverter.h"

#ifdef WITH_CUDA
#include "picking/cuda/Picker_CUDA.h"
#endif

using namespace rigging;

#define DEBUGGING 0

namespace spaint {

//#################### CONSTRUCTORS ####################

TouchSelector::TouchSelector(const Settings_CPtr& settings, const TrackingState_Ptr& trackingState, const View_Ptr& view, size_t maxKeptTouchPoints)
: Selector(settings),
  m_keptTouchPointCount(0),  
  m_keptTouchPointsFloatMB(new ORUtils::MemoryBlock<Vector3f>(maxKeptTouchPoints, true, true)),
  m_keptTouchPointsShortMB(new ORUtils::MemoryBlock<Vector3s>(maxKeptTouchPoints, true, true)),
  m_maxKeptTouchPoints(maxKeptTouchPoints),
  m_touchDetector(new TouchDetector(view->depth->noDims, settings)),
  m_trackingState(trackingState),
  m_view(view)
{
  m_isActive = true;

  // Make the picker.
  if(m_settings->deviceType == ITMLibSettings::DEVICE_CUDA)
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

TouchSelector::ITMUChar4Image_CPtr TouchSelector::generate_touch_image(const View_CPtr& view) const
{
  return m_touchDetector->generate_touch_image(view);
}

std::vector<Eigen::Vector3f> TouchSelector::get_positions() const
{
  // If the last update did not yield any valid touch points, early out.
  if(m_keptTouchPointCount == 0) return std::vector<Eigen::Vector3f>();

  // If the touch points are on the GPU, copy them across to the CPU.
  m_keptTouchPointsFloatMB->UpdateHostFromDevice();

  // Convert the touch points from voxel coordinates into scene coordinates and return them.
  float voxelSize = m_settings->sceneParams.voxelSize;
  const Vector3f *keptTouchPointsFloat = m_keptTouchPointsFloatMB->GetData(MEMORYDEVICE_CPU);
  std::vector<Eigen::Vector3f> touchPoints(m_keptTouchPointCount);
  for(size_t i = 0; i < m_keptTouchPointCount; ++i)
  {
    const Vector3f& keptTouchPoint = keptTouchPointsFloat[i];
    touchPoints[i] = Eigen::Vector3f(keptTouchPoint.x, keptTouchPoint.y, keptTouchPoint.z) * voxelSize;
  }

  return touchPoints;
}

Selector::Selection_CPtr TouchSelector::get_selection() const
{
  return m_keptTouchPointCount > 0 ? m_keptTouchPointsShortMB : Selection_CPtr();
}

void TouchSelector::update(const InputState& inputState, const RenderState_CPtr& renderState, bool renderingInMono)
{
  // Detect any points that the user is touching in the scene.
  MoveableCamera_CPtr camera(new SimpleCamera(CameraPoseConverter::pose_to_camera(*m_trackingState->pose_d)));
  ITMFloatImage_Ptr depthImage(m_view->depth, boost::serialization::null_deleter());
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
