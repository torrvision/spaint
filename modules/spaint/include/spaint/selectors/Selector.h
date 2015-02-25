/**
 * spaint: Selector.h
 */

#ifndef H_SPAINT_SELECTOR
#define H_SPAINT_SELECTOR

#include <boost/shared_ptr.hpp>

#include <ITMLib/Objects/ITMRenderState.h>
#include <ITMLib/Objects/ITMScene.h>

#include "../input/InputState.h"

namespace spaint {

/**
 * \brief An instance of a class deriving from this one can be used to select one or more voxels in the scene.
 */
class Selector
{
  //#################### TYPEDEFS ####################
public:
  typedef boost::shared_ptr<ITMLib::Objects::ITMRenderState> RenderState_CPtr;
  typedef ORUtils::MemoryBlock<Vector3s> Selection;
  typedef boost::shared_ptr<Selection> Selection_Ptr;
  typedef boost::shared_ptr<Selection> Selection_CPtr;

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the selector.
   */
  virtual ~Selector() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Selects some voxels in the scene based on the current input state.
   *
   * \param inputState            The current input state.
   * \param monocularRenderState  The render state for the monocular camera from which the scene is currently being viewed (may be null if a stereo view is being used).
   * \return                      The selected voxels.
   */
  virtual Selection_CPtr select_voxels(const InputState& inputState, const RenderState_CPtr& renderState) const = 0;

  /**
   * \brief Updates the selector based on the current input state (this is how we allow the user to control the parameters of the selector).
   *
   * \param inputState  The current input state.
   */
  virtual void update(const InputState& inputState) = 0;
};

}

#endif
