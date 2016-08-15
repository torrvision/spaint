/**
 * spaintgui: SLAMState.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_SLAMSTATE
#define H_SPAINTGUI_SLAMSTATE

#include <RelocLib/PoseDatabase.h>
#include <RelocLib/Relocaliser.h>

#include <spaint/trackers/FallibleTracker.h>

#include "Raycaster.h"

/**
* \brief TODO
*/
class SLAMState
{
  //#################### TYPEDEFS ####################
protected:
  typedef boost::shared_ptr<RelocLib::PoseDatabase> PoseDatabase_Ptr;
  typedef boost::shared_ptr<RelocLib::Relocaliser> Relocaliser_Ptr;

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
  virtual const Model_Ptr& get_model() const = 0;

  /**
   * \brief TODO
   */
  virtual const PoseDatabase_Ptr& get_pose_database() const = 0;

  /**
   * \brief TODO
   */
  virtual const Raycaster_Ptr& get_raycaster() const = 0;

  /**
   * \brief TODO
   */
  virtual const Relocaliser_Ptr& get_relocaliser() const = 0;
};

#endif
