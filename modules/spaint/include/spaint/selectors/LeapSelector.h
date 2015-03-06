/**
 * spaint: LeapSelector.h
 */

#ifndef H_SPAINT_LEAPSELECTOR
#define H_SPAINT_LEAPSELECTOR

#include <Eigen/Dense>

#include <ITMLib/Objects/ITMScene.h>

// Note: This #undef is a disgusting hack that is needed to work around the fact that InfiniTAM #defines PI in a header.
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
  typedef ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> Scene;
  typedef boost::shared_ptr<const Scene> Scene_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The most recent frame of data from the Leap Motion. */
  Leap::Frame m_frame;

  /** The Leap Motion controller. */
  Leap::Controller m_leap;

  /** A memory block into which to store the most recent point picked by the user as a Vector3s, in voxel coordinates. */
  // FIXME: This is the same as in PickingSelector - we should factor out the common code.
  ORUtils::MemoryBlock<Vector3s> m_pickPointShortMB;

  /** The InfiniTAM scene. */
  Scene_CPtr m_scene;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a Leap selector.
   *
   * \param settings  The settings to use for InfiniTAM.
   * \param scene     The InfiniTAM scene.
   */
  LeapSelector(const Settings_CPtr& settings, const Scene_CPtr& scene);

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
  virtual void update(const InputState& inputState, const RenderState_CPtr& renderState);

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Converts a vector in the Leap coordinate system into one in our coordinate system.
   *
   * \param leapVec The vector in the Leap coordinate system.
   * \return        The vector in our coordinate system.
   */
  static Eigen::Vector3f from_leap_vector(const Leap::Vector& leapVec);
};

}

#endif
