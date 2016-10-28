/**
 * spaint: TouchSelector.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_TOUCHSELECTOR
#define H_SPAINT_TOUCHSELECTOR

#include <Eigen/Dense>

#include "Selector.h"
#include "../picking/interface/Picker.h"
#include "../touch/TouchDetector.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to select voxels in the scene based on what the user is touching in the real world.
 *
 * This is achieved by analysing the differences between the live depth input and a depth raycast of the scene.
 */
class TouchSelector : public Selector
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<TouchDetector> TouchDetector_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The number of touch points that were kept in the most recent update. */
  size_t m_keptTouchPointCount;

  /** A memory block into which to store the kept touch points in Vector3f format and voxel coordinates. */
  mutable boost::shared_ptr<ORUtils::MemoryBlock<Vector3f> > m_keptTouchPointsFloatMB;

  /** A selection into which to store the kept touch points in Vector3s format and voxel coordinates. */
  Selection_Ptr m_keptTouchPointsShortMB;

  /** The maximum number of touch points that we should keep in a single update (we limit this for performance reasons). */
  size_t m_maxKeptTouchPoints;

  /** The picker. */
  boost::shared_ptr<const Picker> m_picker;

  /** The touch detector. */
  TouchDetector_Ptr m_touchDetector;

  //#################### CONSTRUCTORS ####################
public:
  /*
   * \brief Constructs a touch selector.
   *
   * \param itmSettings         The settings to use for InfiniTAM.
   * \param touchSettings       The settings to use for the touch detector.
   * \param touchImageSize      The size of the images on which the touch detector is to run.
   * \param maxKeptTouchPoints  The maximum number of touch points that we should keep in a single update (we limit this for performance reasons).
   */
  TouchSelector(const Settings_CPtr& itmSettings, const TouchSettings_Ptr& touchSettings, const Vector2i& touchImageSize, size_t maxKeptTouchPoints);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void accept(const SelectorVisitor& visitor) const;

  /**
   * \brief Generates a colour image containing the current touch interaction (if any).
   *
   * \param view  The current view.
   * \return      A colour image containing the current touch interaction (if any).
   */
  ITMUChar4Image_CPtr generate_touch_image(const View_CPtr& view) const;

  /**
   * \brief Gets the positions of the current touch points.
   *
   * \return The positions of the current touch points.
   */
  std::vector<Eigen::Vector3f> get_positions() const;

  /** Override */
  virtual Selection_CPtr get_selection() const;

  /** Override */
  virtual void update(const tvginput::InputState& inputState, const SLAMState_CPtr& slamState, const VoxelRenderState_CPtr& renderState, bool renderingInMono);
};

}

#endif
