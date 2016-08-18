/**
 * spaintgui: SLAMState.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_SLAMSTATE
#define H_SPAINTGUI_SLAMSTATE

#include "Raycaster.h"

/**
* \brief TODO
*/
class SLAMState
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<ITMLib::ITMView> View_Ptr;

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief TODO
   */
  virtual ~SLAMState() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  virtual const Raycaster_Ptr& get_raycaster() const = 0;

  /**
   * \brief TODO
   */
  virtual const View_Ptr& get_view() const = 0;

  /**
   * \brief TODO
   */
  virtual void set_view(ITMLib::ITMView *view) = 0;
};

#endif
