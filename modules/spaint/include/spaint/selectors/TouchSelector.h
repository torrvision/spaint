/**
 * spaint: TouchSelector.h
 */

#ifndef H_SPAINT_TOUCHSELECTOR
#define H_SPAINT_TOUCHSELECTOR

#include <Eigen/Dense>

#include <ITMLib/Objects/ITMTrackingState.h>
#include <ITMLib/Objects/ITMView.h>

#include "Selector.h"
#include "../picking/interface/Picker.h"
#include "../touch/TouchDetector.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to select voxels in the scene based on what theuser is touching in the real world.
 *
 * This is achieved by analysing the differences between the live depth input and a depth raycast of the scene.
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
  mutable boost::shared_ptr<ORUtils::MemoryBlock<Vector3f> > m_pickPointFloatMB;

  /** A selection into which to store the most recent point picked by the user as a Vector3s, in voxel coordinates. */
  Selection_Ptr m_pickPointShortMB;

  /** Whether or not the most recent update operation returned at least one valid pick point. */
  bool m_pickPointValid;

  /** The maximum number of pick-points allowed. */
  int m_maximumValidPickPoints;

  /** The number of valid pick-points. */
  int m_numberOfValidPickPoints;

  /** The touch detector. */
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
   * \param settings       The settings to use for InfiniTAM.
   * \param trachingState  The InfiniTAM tracking state which contains the pose5of the camera.
   * \param view           The InfiniTAM view which contains the raw depth image.
   */
  explicit TouchSelector(const Settings_CPtr& settings, const TrackingState_Ptr& trackingState, const View_Ptr& view);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void accept(const SelectorVisitor& visitor) const;

  /**
   * \brief Gets the positions of the touch points (if known).
   *
   * \return The positions of the touch points (if known), or boost::none otherwise.
   */
  std::vector<Eigen::Vector3f> get_positions() const;

  /** Override */
  virtual Selection_CPtr get_selection() const;

  /** Override */
  virtual void update(const InputState& inputState, const RenderState_CPtr& renderState);
};

}

#endif

