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

#include <rigging/MoveableCamera.h>

#include "Selector.h"
#include "../picking/interface/Picker.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to select voxels in the scene using the Leap Motion.
 */
class LeapSelector : public Selector
{
  //#################### ENUMERATIONS ####################
public:
  /**
   * \brief The values of this enumeration denote the different modes in which a Leap selector can operate.
   */
  enum Mode
  {
    /** In point mode, the user selects voxels in the scene by pointing at them. */
    MODE_POINT,

    /** In touch mode, the user selects voxels in the scene by touching them. */
    MODE_TOUCH
  };

  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ITMLib::ITMVisualisationEngine<SpaintVoxel,ITMVoxelIndex> > VoxelVisualisationEngine_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The camera representing the Leap Motion controller's coordinate frame. */
  rigging::MoveableCamera_Ptr m_camera;

  /** The ID of the fiducial (if any) from which to obtain the Leap Motion controller's coordinate frame. */
  std::string m_fiducialID;

  /** The most recent frame of data from the Leap Motion. */
  Leap::Frame m_frame;

  /** The Leap Motion controller. */
  Leap::Controller m_leap;

  /** The mode in which the selector is operating. */
  Mode m_mode;

  /** The picker. */
  Picker_CPtr m_picker;

  /** A memory block into which to store the most recent point picked by the user as a Vector3f, in voxel coordinates. */
  boost::shared_ptr<ORUtils::MemoryBlock<Vector3f> > m_pickPointFloatMB;

  /** A selection into which to store the most recent point picked by the user as a Vector3s, in voxel coordinates. */
  Selection_Ptr m_pickPointShortMB;

  /** Whether or not the most recent update operation returned a valid pick point. */
  bool m_pickPointValid;

  /** The InfiniTAM engine used for rendering a voxel scene. */
  VoxelVisualisationEngine_CPtr m_visualisationEngine;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a Leap selector.
   *
   * \param settings            The settings to use for InfiniTAM.
   * \param visualisationEngine The InfiniTAM engine used for rendering a voxel scene.
   * \param mode                The mode in which the selector should operate.
   * \param fiducialID          The ID of the fiducial (if any) from which to obtain the Leap Motion controller's coordinate frame.
   */
  LeapSelector(const Settings_CPtr& settings, const VoxelVisualisationEngine_CPtr& visualisationEngine, Mode mode, const std::string& fiducialID = "");

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void accept(const SelectorVisitor& visitor) const;

  /**
   * \brief Converts a free vector in the Leap coordinate system into one in the InfiniTAM coordinate system.
   *
   * \param leapDir The free vector in the Leap coordinate system.
   * \return        The free vector in the InfiniTAM coordinate system.
   */
  Eigen::Vector3f from_leap_direction(const Leap::Vector& leapDir) const;

  /**
   * \brief Converts a position vector in the Leap coordinate system into one in the InfiniTAM coordinate system.
   *
   * \param leapPos The position vector in the Leap coordinate system.
   * \return        The position vector in the InfiniTAM coordinate system.
   */
  Eigen::Vector3f from_leap_position(const Leap::Vector& leapPos) const;

  /**
   * \brief Gets the camera representing the Leap Motion controller's coordinate frame.
   *
   * \return  The camera representing the Leap Motion controller's coordinate frame.
   */
  const rigging::Camera& get_camera() const;

  /**
   * \brief Gets the most recent frame of data from the Leap Motion.
   *
   * \return  The most recent frame of data from the Leap Motion.
   */
  const Leap::Frame& get_frame() const;

  /**
   * \brief Gets the mode in which the selector is operating.
   *
   * \return  The mode in which the selector is operating.
   */
  Mode get_mode() const;

  /**
   * \brief Gets the selected position (if known).
   *
   * \return  The selected position (if known), or boost::none otherwise.
   */
  boost::optional<Eigen::Vector3f> get_position() const;

  /** Override */
  virtual Selection_CPtr get_selection() const;

  /** Override */
  virtual void update(const tvginput::InputState& inputState, const SLAMState_CPtr& slamState, const VoxelRenderState_CPtr& renderState, bool renderingInMono);

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
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
