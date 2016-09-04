/**
 * spaint: LeapSelector.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_LEAPSELECTOR
#define H_SPAINT_LEAPSELECTOR

#include <Eigen/Dense>

// This #undef is a disgusting hack that is needed to work around the fact that InfiniTAM #defines PI in a header.
// Since the Leap SDK defines PI as a float (static const float PI = ...), things would break if we didn't do this.
#undef PI

#include <Leap.h>

#include <ITMLib/Engines/Visualisation/Interface/ITMVisualisationEngine.h>

#include "Selector.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to select voxels in the scene using the Leap Motion.
 */
class LeapSelector : public Selector
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ITMLib::ITMVisualisationEngine<SpaintVoxel,ITMVoxelIndex> > VoxelVisualisationEngine_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The most recent frame of data from the Leap Motion. */
  Leap::Frame m_frame;

  /** The Leap Motion controller. */
  Leap::Controller m_leap;

  /** A selection into which to store the most recent point picked by the user as a Vector3s, in voxel coordinates. */
  Selection_Ptr m_pickPointShortMB;

  /** The InfiniTAM engine used for rendering a voxel scene. */
  VoxelVisualisationEngine_CPtr m_visualisationEngine;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a Leap selector.
   *
   * \param settings            The settings to use for InfiniTAM.
   * \param visualisationEngine The InfiniTAM engine used for rendering a voxel scene.
   */
  LeapSelector(const Settings_CPtr& settings, const VoxelVisualisationEngine_CPtr& visualisationEngine);

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
  virtual void update(const tvginput::InputState& inputState, const SLAMState_CPtr& slamState, const VoxelRenderState_CPtr& renderState, bool renderingInMono);

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Converts a free vector in the Leap coordinate system into one in the InfiniTAM coordinate system.
   *
   * \param leapDir The free vector in the Leap coordinate system.
   * \return        The free vector in the InfiniTAM coordinate system.
   */
  static Eigen::Vector3f from_leap_direction(const Leap::Vector& leapDir);

  /**
   * \brief Converts a position vector in the Leap coordinate system into one in the InfiniTAM coordinate system.
   *
   * \param leapPos The position vector in the Leap coordinate system.
   * \return        The position vector in the InfiniTAM coordinate system.
   */
  static Eigen::Vector3f from_leap_position(const Leap::Vector& leapPos);

  /**
   * \brief Converts a size in the Leap coordinate system into one in the InfiniTAM coordinate system.
   *
   * \param leapSize  The size in the Leap coordinate system.
   * \return          The size in the InfiniTAM coordinate system.
   */
  static float from_leap_size(float leapSize);
};

}

#endif
