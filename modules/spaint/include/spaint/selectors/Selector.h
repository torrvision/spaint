/**
 * spaint: Selector.h
 */

#ifndef H_SPAINT_SELECTOR
#define H_SPAINT_SELECTOR

#include <boost/shared_ptr.hpp>

#include <ITMLib/Objects/ITMRenderState.h>
#include <ITMLib/Objects/ITMScene.h>

#include "SelectorVisitor.h"
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
   * \brief Accepts a visitor.
   *
   * \param visitor The visitor to accept.
   */
  virtual void accept(const SelectorVisitor& visitor) const = 0;

  /**
   * \brief Selects some voxels in the scene.
   *
   * \return  The selected voxels.
   */
  virtual Selection_CPtr select_voxels() const = 0;

  /**
   * \brief Updates the selector based on the current input state.
   *
   * \param inputState  The current input state.
   * \param renderState The render state corresponding to the camera from which the scene is being viewed.
   */
  virtual void update(const InputState& inputState, const RenderState_CPtr& renderState) = 0;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<Selector> Selector_Ptr;
typedef boost::shared_ptr<const Selector> Selector_CPtr;

}

#endif
