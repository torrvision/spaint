/**
 * spaint: PickingSelector.h
 */

#ifndef H_SPAINT_PICKINGSELECTOR
#define H_SPAINT_PICKINGSELECTOR

#include <boost/optional.hpp>

#include <Eigen/Dense>

#include <ITMLib/Utils/ITMLibSettings.h>

#include "Selector.h"
#include "../picking/interface/Picker.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to select a cube of voxels in the scene using picking.
 */
class PickingSelector : public Selector
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<ITMFloat4Image> Float4Image_Ptr;
  typedef boost::shared_ptr<const ITMLibSettings> Settings_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The picker. */
  boost::shared_ptr<const Picker> m_picker;

  /** A memory block into which to store the most recent point picked by the user as a Vector3f, in voxel coordinates. */
  mutable ORUtils::MemoryBlock<Vector3f> m_pickPointFloatMB;

  /** A memory block into which to store the most recent point picked by the user as a Vector3s, in voxel coordinates. */
  mutable ORUtils::MemoryBlock<Vector3s> m_pickPointShortMB;

  /** Whether or not the most recent picking operation returned a valid pick point. */
  bool m_pickPointValid;

  /** The selection radius (we select all voxels in a cube of side length 2 * radius + 1, centered on the voxel the user actually clicks). */
  int m_radius;

  /** The settings to use for InfiniTAM. */
  Settings_CPtr m_settings;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a picking selector.
   *
   * \param settings  The settings to use for InfiniTAM.
   */
  explicit PickingSelector(const Settings_CPtr& settings);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void accept(const SelectorVisitor& visitor) const;

  /**
   * \brief Gets the position of the selector (if known).
   *
   * \return  The position of the selector (if known), or boost::none otherwise.
   */
  boost::optional<Eigen::Vector3f> get_position() const;

  /**
   * \brief Gets the selection radius.
   *
   * \return  The selection radius.
   */
  int get_radius() const;

  /** Override */
  virtual Selection_CPtr select_voxels() const;

  /** Override */
  virtual void update(const InputState& inputState, const RenderState_CPtr& renderState);
};

}

#endif
