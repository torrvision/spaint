/*
 * spaint: TouchSelector.cpp
 */

#include "selectors/TouchSelector.h"

#include <tvgutil/timers/Timer.h>

#include "picking/cpu/Picker_CPU.h"
#ifdef WITH_CUDA
#include "picking/cuda/Picker_CUDA.h"
#endif
#include "util/CameraPoseConverter.h"

using namespace rigging;

namespace spaint {

//#################### CONSTRUCTORS ####################

TouchSelector::TouchSelector(const Settings_CPtr& settings, const TrackingState_Ptr& trackingState, const View_Ptr& view)
: Selector(settings),
  m_pickPointValid(false),
  m_maximumValidPickPoints(50),
  m_numberOfValidPickPoints(0),
  m_touchDetector(new TouchDetector(view->depth->noDims)),
  m_trackingState(trackingState),
  m_view(view)
{
  m_pickPointFloatMB.reset(new ORUtils::MemoryBlock<Vector3f>(m_maximumValidPickPoints, true, true));
  m_pickPointShortMB.reset(new ORUtils::MemoryBlock<Vector3s>(m_maximumValidPickPoints, true, true));

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

std::vector<Eigen::Vector3f> TouchSelector::get_positions() const
{
  // If the last update did not yield a valid pick point, early out.
  if(!m_pickPointValid) return std::vector<Eigen::Vector3f>();

  int nPickPoints = std::min(m_numberOfValidPickPoints, m_maximumValidPickPoints);
  std::vector<Eigen::Vector3f> pickPoints(nPickPoints);

  // If the pick point is onthe GPU, copy it across to the CPU.
  if(m_settings->deviceType == ITMLibSettings::DEVICE_CUDA) m_pickPointFloatMB->UpdateHostFromDevice();

  // Convert the pick points from voxel coordinates into scene coordinates and return it.
  float voxelSize = m_settings->sceneParams.voxelSize;
  for(int i = 0; i < nPickPoints; ++i)
  {
    const Vector3f& pickPoint = *m_pickPointFloatMB->GetData(MEMORYDEVICE_CPU)[i];
    pickPoints[i] = Eigen::Vector3f(pickPoint.x * voxelSize, pickPoint.y * voxelSize, pickPoint.z * voxelSize);
  }
  return pickPoints;
}

Selector::Selection_CPtr TouchSelector::get_selection() const
{
  return m_pickPointValid ? m_pickPointShortMB : Selection_CPtr();
}

void TouchSelector::update(const InputState& inputState, const RenderState_CPtr& renderState)
{
  // Get camera.
  static boost::shared_ptr<SimpleCamera> camera;
  camera.reset( new SimpleCamera(CameraPoseConverter::pose_to_camera(*m_trackingState->pose_d)));

  // Get voxel size.
  static float voxelSize = m_settings->sceneParams.voxelSize;

  // Run the touch pipeline.
  TIME(m_touchDetector->run_touch_detector_on_frame(renderState, camera, voxelSize, m_view->depth),milliseconds, runningTouchDetectorOnFrame);
  std::cout << runningTouchDetectorOnFrame << '\n';

  // Get the touch state.
  const TouchState& touchState = m_touchDetector->get_touch_state();

  // Update whether or not the selector is active.
  m_isActive = touchState.touching_surface();

  // Try and pick an individual voxel.
  m_pickPointValid = false;
  m_numberOfValidPickPoints = 0;

  if(!touchState.touch_position_known()) return;
  const std::vector<int>& pointsx = touchState.position_x();
  const std::vector<int>& pointsy = touchState.position_y();
  int nTouchPoints = pointsx.size();

  // FIXME: Instead of clearing the MemoryBlock at each frame, pass around it's size. Resizeable with fixed length back buffer.
  if(nTouchPoints < m_maximumValidPickPoints)
    m_pickPointFloatMB.reset(new ORUtils::MemoryBlock<Vector3f>(m_maximumValidPickPoints, true, true));

  Vector3f *m_pickPointFloatMBData = m_pickPointFloatMB->GetData(MEMORYDEVICE_CPU);

  //FIXME Only selecting the forest point for now.
  for(int i = 0; i < nTouchPoints; ++i)
  {
    bool pickPointValid = false;
    ORUtils::MemoryBlock<Vector3f> pickPointFloatMB(1, true, true);
    pickPointValid = m_picker->pick(pointsx[i], pointsy[i], renderState.get(), pickPointFloatMB);
    if(pickPointValid)
    {
      pickPointFloatMB.UpdateHostFromDevice();
      Vector3f * pickPointFloatMBData = pickPointFloatMB.GetData(MEMORYDEVICE_CPU);
      m_pickPointFloatMBData[i] = Vector3f(pickPointFloatMBData->x, pickPointFloatMBData->y, pickPointFloatMBData->z);
      ++m_numberOfValidPickPoints;
    }
    if(m_numberOfValidPickPoints >= m_maximumValidPickPoints)
      break;
  }

  if(m_numberOfValidPickPoints > 0)
  {
    m_pickPointValid = true;
    m_pickPointFloatMB->UpdateDeviceFromHost();
/*    {
      Vector3f *tmp = m_pickPointFloatMB->GetData(MEMORYDEVICE_CPU);
      int size = m_pickPointFloatMB->dataSize;
      for(int i = 0; i < size; ++i)
      {
        std::cout << "x=" << tmp[i].x << " y=" << tmp[i].y << " z=" << tmp[i].z << '\n';
      }
    }
*/
    m_picker->to_short(*m_pickPointFloatMB, *m_pickPointShortMB);

/*    m_pickPointShortMB->UpdateHostFromDevice();
    {
      Vector3s *tmp = m_pickPointShortMB->GetData(MEMORYDEVICE_CPU);
      int size = m_pickPointShortMB->dataSize;
      for(int i = 0; i < size; ++i)
      {
        std::cout << "x=" << tmp[i].x << " y=" << tmp[i].y << " z=" << tmp[i].z << '\n';
      }
    }
*/
  }
  else
  {
    m_pickPointValid = false;
  }
}

}
