/**
 * spaint: NullSelector.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
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
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a null selector.
   *
   * \param settings  The settings to use for InfiniTAM.
   */
  explicit NullSelector(const Settings_CPtr& settings);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void accept(const SelectorVisitor& visitor) const;

  /** Override */
  virtual Selection_CPtr get_selection() const;

  /** Override */
  virtual void update(const tvginput::InputState& inputState, const RenderState_CPtr& renderState, bool renderingInMono);
};

}

#endif
