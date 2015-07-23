/**
 * spaint: PickingSelector.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
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
  //#################### PRIVATE VARIABLES ####################
private:
  /** The picker. */
  boost::shared_ptr<const Picker> m_picker;

  /** A memory block into which to store the most recent point picked by the user as a Vector3f, in voxel coordinates. */
  boost::shared_ptr<ORUtils::MemoryBlock<Vector3f> > m_pickPointFloatMB;

  /** A selection into which to store the most recent point picked by the user as a Vector3s, in voxel coordinates. */
  Selection_Ptr m_pickPointShortMB;

  /** Whether or not the most recent update operation returned a valid pick point. */
  bool m_pickPointValid;

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

  /** Override */
  virtual Selection_CPtr get_selection() const;

  /** Override */
  virtual void update(const InputState& inputState, const RenderState_CPtr& renderState, bool renderingInMono);
};

}

#endif
