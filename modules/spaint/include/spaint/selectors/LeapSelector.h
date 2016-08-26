/**
 * spaint: LeapSelector.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_LEAPSELECTOR
#define H_SPAINT_LEAPSELECTOR

#include <Eigen/Dense>

#include <ITMLib/Objects/Scene/ITMScene.h>

// This #undef is a disgusting hack that is needed to work around the fact that InfiniTAM #defines PI in a header.
// Since the Leap SDK defines PI as a float (static const float PI = ...), things would break if we didn't do this.
#undef PI

#include <Leap.h>

#include "Selector.h"
#include "../util/SpaintVoxel.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to select voxels in the scene using the Leap Motion.
 */
class LeapSelector : public Selector
{
  //#################### TYPEDEFS ####################
private:
  typedef ITMLib::ITMScene<SpaintVoxel,ITMVoxelIndex> Scene;
  typedef boost::shared_ptr<const Scene> Scene_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The most recent frame of data from the Leap Motion. */
  Leap::Frame m_frame;

  /** The Leap Motion controller. */
  Leap::Controller m_leap;

  /** A selection into which to store the most recent point picked by the user as a Vector3s, in voxel coordinates. */
  Selection_Ptr m_pickPointShortMB;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a Leap selector.
   *
   * \param settings  The settings to use for InfiniTAM.
   */
  explicit LeapSelector(const Settings_CPtr& settings);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void accept(const SelectorVisitor& visitor) const;

  /**
   * \brief Gets the most recent frame of data from the Leap Motion.
   *
   * \return  The most recent frame of data from the Leap Motion.
   */
  const Leap::Frame& get_frame() const;

  /** Override */
  virtual Selection_CPtr get_selection() const;

  /** Override */
  virtual void update(const tvginput::InputState& inputState, const VoxelRenderState_CPtr& renderState, bool renderingInMono);

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Converts a size in the Leap coordinate system into one in the InfiniTAM coordinate system.
   *
   * \param leapSize  The size in the Leap coordinate system.
   * \return          The size in the InfiniTAM coordinate system.
   */
  static float from_leap_size(float leapSize);

  /**
   * \brief Converts a vector in the Leap coordinate system into one in the InfiniTAM coordinate system.
   *
   * \param leapVec The vector in the Leap coordinate system.
   * \return        The vector in the InfiniTAM coordinate system.
   */
  static Eigen::Vector3f from_leap_vector(const Leap::Vector& leapVec);
};

}

#endif
