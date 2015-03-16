/**
 * spaint: TouchSelector.h
 */

#ifndef H_SPAINT_TOUCHSELECTOR
#define H_SPAINT_TOUCHSELECTOR

#include <boost/optional.hpp>

#include <Eigen/Dense>

#include <ITMLib/Objects/ITMTrackingState.h>
#include <ITMLib/Objects/ITMView.h>

#include "PickingSelector.h"
#include "../touch/TouchDetector.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to select a cube of voxels in the scene using touch.
 */
class TouchSelector : public Selector
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<TouchDetector> TouchDetector_Ptr;
  typedef boost::shared_ptr<ITMTrackingState> TrackingState_Ptr;
  typedef boost::shared_ptr<ITMView> View_Ptr;

  //#################### PRIVATE VARIABLES #################### 
private:
  /** The picker. */
  boost::shared_ptr<const Picker> m_picker;

  /** A memory block into which to store the most recent point picked by the user as a Vector3f, in voxel coordinates. */
  mutable ORUtils::MemoryBlock<Vector3f> m_pickPointFloatMB;

  /** A selection into which to store the most recent point picked by the user as a Vector3s, in voxel coordinates. */
  Selection_Ptr m_pickPointShortMB;

  /** Whether or not the most recent update operation returned a valid pick point. */
  bool m_pickPointValid;

  /** A touch detector. */
  TouchDetector_Ptr m_touchDetector;

  /** The traching state. */
  TrackingState_Ptr m_trackingState;

  /** The view. */
  View_Ptr m_view;

  //#################### CONSTRUCTORS #################### 
public:
  /*
   * \brief Constructs a touch selector.
   *
   * \param settings  The settings to use for InfiniTAM.
   */
  explicit TouchSelector(const Settings_CPtr& settings, const TrackingState_Ptr& trackingState, const View_Ptr& view);

  //#################### PUBLIC MEMBER FUNCTIONS #################### 
public:
  /** Override */
  virtual void accept(const SelectorVisitor& visitor) const;

  /**
   * \brief Gets the position of the selector (if known).
   *
   * \return The position of the selector (if known), or boost::none otherwise.
   */
  boost::optional<Eigen::Vector3f> get_position() const;

  /** Override */
  virtual Selection_CPtr get_selection() const;

  /** Override */
  virtual void update(const InputState& inputState, const RenderState_CPtr& renderState);
};

}

#endif

