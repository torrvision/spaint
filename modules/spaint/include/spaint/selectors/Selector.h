/**
 * spaint: Selector.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_SELECTOR
#define H_SPAINT_SELECTOR

#include <boost/shared_ptr.hpp>

#include <ITMLib/Objects/RenderStates/ITMRenderState.h>
#include <ITMLib/Objects/Scene/ITMScene.h>
#include <ITMLib/Utils/ITMLibSettings.h>

#include <tvginput/InputState.h>

#include "SelectorVisitor.h"

namespace spaint {

/**
 * \brief An instance of a class deriving from this one can be used to select one or more voxels in the scene.
 */
class Selector
{
  //#################### TYPEDEFS ####################
public:
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;
  typedef ORUtils::MemoryBlock<Vector3s> Selection;
  typedef boost::shared_ptr<Selection> Selection_Ptr;
  typedef boost::shared_ptr<const Selection> Selection_CPtr;
  typedef boost::shared_ptr<const ITMLib::ITMLibSettings> Settings_CPtr;

  //#################### PROTECTED VARIABLES ####################
protected:
  /** Whether or not the selector is active. */
  bool m_isActive;

  /** The settings to use for InfiniTAM. */
  Settings_CPtr m_settings;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a selector.
   *
   * \param settings  The settings to use for InfiniTAM.
   */
  explicit Selector(const Settings_CPtr& settings)
  : m_isActive(false), m_settings(settings)
  {}

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the selector.
   */
  virtual ~Selector() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Accepts a visitor.
   *
   * \param visitor The visitor to accept.
   */
  virtual void accept(const SelectorVisitor& visitor) const = 0;

  /**
   * \brief Gets the voxels in the scene (if any) that were selected by the last update.
   *
   * \return  The voxels in the scene (if any) that were selected by the last update.
   */
  virtual Selection_CPtr get_selection() const = 0;

  /**
   * \brief Gets whether or not the selector is active.
   *
   * \return  true, if the selector is active, or false otherwise.
   */
  bool is_active() const
  {
    return m_isActive;
  }

  /**
   * \brief Updates the selector based on the current input state.
   *
   * \param inputState      The current input state.
   * \param renderState     The render state corresponding to the camera from which the scene is being viewed.
   * \param renderingInMono A flag indicating whether or not the scene is currently being rendered in mono.
   */
  virtual void update(const tvginput::InputState& inputState, const RenderState_CPtr& renderState, bool renderingInMono) = 0;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<Selector> Selector_Ptr;
typedef boost::shared_ptr<const Selector> Selector_CPtr;

}

#endif
