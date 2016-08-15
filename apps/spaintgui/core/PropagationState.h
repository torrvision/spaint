/**
 * spaintgui: PropagationState.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_PROPAGATIONSTATE
#define H_SPAINTGUI_PROPAGATIONSTATE

#include "Interactor.h"

/**
 * \brief TODO
 */
class PropagationState
{
  //#################### DESTRUCTOR ####################
public:
  virtual ~PropagationState() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  virtual const Interactor_Ptr& get_interactor() const = 0;

  /**
   * \brief TODO
   */
  virtual const Model_Ptr& get_model() const = 0;
};

#endif
