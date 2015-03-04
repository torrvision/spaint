/**
 * spaint: LeapSelector.h
 */

#ifndef H_SPAINT_LEAPSELECTOR
#define H_SPAINT_LEAPSELECTOR

#include <Leap.h>

#include "Selector.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to select voxels in the scene using the Leap Motion.
 */
class LeapSelector : public Selector
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The most recent frame of data from the Leap Motion. */
  Leap::Frame m_frame;

  /** The Leap Motion controller. */
  Leap::Controller m_leap;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void accept(const SelectorVisitor& visitor) const;

  /** Override */
  virtual Selection_CPtr get_selection() const;

  /** Override */
  virtual void update(const InputState& inputState, const RenderState_CPtr& renderState);
};

}

#endif
