/**
 * spaint: CollaborativeContext.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_SPAINT_COLLABORATIVECONTEXT
#define H_SPAINT_COLLABORATIVECONTEXT

#include <itmx/relocalisation/RefiningRelocaliser.h>

#include "../collaboration/CollaborativePoseOptimiser.h"
#include "../slamstate/SLAMState.h"

namespace spaint {

/**
 * \brief An instance of a class deriving from this one provides the shared context needed by a collaborative component.
 */
class CollaborativeContext
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The collaborative pose optimiser. */
  CollaborativePoseOptimiser_Ptr m_collaborativePoseOptimiser;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a collaborative context.
   */
  CollaborativeContext();

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  virtual itmx::RefiningRelocaliser_Ptr& get_relocaliser(const std::string& sceneID) = 0;
  virtual itmx::RefiningRelocaliser_CPtr get_relocaliser(const std::string& sceneID) const = 0;
  virtual const std::vector<std::string>& get_scene_ids() const = 0;
  virtual const Settings_CPtr& get_settings() const = 0;
  virtual SLAMState_CPtr get_slam_state(const std::string& sceneID) const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  virtual const CollaborativePoseOptimiser_Ptr& get_collaborative_pose_optimiser();
  virtual CollaborativePoseOptimiser_CPtr get_collaborative_pose_optimiser() const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<CollaborativeContext> CollaborativeContext_Ptr;

}

#endif
