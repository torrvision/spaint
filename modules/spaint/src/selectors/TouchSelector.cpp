/*
 * spaint: TouchSelector.cpp
 */

#include <tvgutil/timers/Timer.h>

#include "selectors/TouchSelector.h"

#include "picking/cpu/Picker_CPU.h"

#include "util/CameraPoseConverter.h"

#ifdef WITH_CUDA
#include "picking/cuda/Picker_CUDA.h"
#endif

namespace spaint {

//#################### CONSTRUCTORS #################### 

TouchSelector::TouchSelector(const Settings_CPtr& settings, const TrackingState_Ptr& trackingState, const View_Ptr& view)
: Selector(settings), 
  m_pickPointFloatMB(1, true, true),
  m_pickPointShortMB(new ORUtils::MemoryBlock<Vector3s>(1, true, true)),
  m_pickPointValid(false),
  m_touchDetector(new TouchDetector(view->depth->noDims)), 
  m_trackingState(trackingState), 
  m_view(view)
{
  //Make the picker. 
  if(m_settings->deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    m_picker.reset(new Picker_CUDA);
#else
    //This should never happen as things stand - we set deviceType to DEVICE_CPU if CUDA support isn't available.
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

boost::optional<Eigen::Vector3f> TouchSelector::get_position() const
{
  // If the last update did not yield a valid pick point, early out.
  if(!m_pickPointValid) return boost::none;

  // If the pick point is onthe GPU, copy it across to the CPU.
  if(m_settings->deviceType == ITMLibSettings::DEVICE_CUDA) m_pickPointFloatMB.UpdateHostFromDevice();

  // Convert the pick point from voxel coordinates into scene coordinates and return it.
  float voxelSize = m_settings->sceneParams.voxelSize;
  const Vector3f& pickPoint = *m_pickPointFloatMB.GetData(MEMORYDEVICE_CPU);
  return Eigen::Vector3f(pickPoint.x * voxelSize, pickPoint.y * voxelSize, pickPoint.z * voxelSize);
}

Selector::Selection_CPtr TouchSelector::get_selection() const
{
  return m_pickPointValid ? m_pickPointShortMB : Selection_CPtr();
}

void TouchSelector::update(const InputState& inputState, const RenderState_CPtr& renderState)
{
  // Run the touch pipeline.
  static boost::shared_ptr<rigging::SimpleCamera> camera(new rigging::SimpleCamera(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()));
  camera->set_from(CameraPoseConverter::pose_to_camera(*m_trackingState->pose_d));
  static float voxelSize = m_settings->sceneParams.voxelSize;
  //static const ITMIntrinsics& intrinsics = m_view->calib->intrinsics_d;

  TIME(m_touchDetector->run_touch_detector_on_frame(renderState, camera, voxelSize, m_view->depth),milliseconds, runningTouchDetectorOnFrame);
  std::cout << runningTouchDetectorOnFrame << '\n';
  const TouchState& touchState = m_touchDetector->get_touch_state();

  // Update whether or not the selector is active.
  m_isActive = touchState.touching_surface();

  // Try and pick an individual voxel.
  m_pickPointValid = false;

  if(!touchState.touch_position_known()) return;
  const std::vector<int>& pointsx = touchState.position_x();
  const std::vector<int>& pointsy = touchState.position_y();

  //FIXME Only selecting the forest point for now.
  m_pickPointValid = m_picker->pick(pointsx[0], pointsy[0], renderState.get(), m_pickPointFloatMB);
  if(m_pickPointValid) m_picker->to_short(m_pickPointFloatMB, *m_pickPointShortMB);
}

}
