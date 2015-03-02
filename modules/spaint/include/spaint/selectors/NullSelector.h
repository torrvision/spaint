/**
 * spaint: NullSelector.h
 */

#ifndef H_SPAINT_NULLSELECTOR
#define H_SPAINT_NULLSELECTOR

#include "Selector.h"

namespace spaint {

/**
 * \brief An instance of this class can be used when we're not using any of the other selectors.
 */
class NullSelector : public Selector
{
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
