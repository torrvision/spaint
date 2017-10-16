/**
 * spaint: CollaborativeContext.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_SPAINT_COLLABORATIVECONTEXT
#define H_SPAINT_COLLABORATIVECONTEXT

#include <itmx/relocalisation/RefiningRelocaliser.h>

#include "../collaboration/PoseGraphOptimiser.h"
#include "../slamstate/SLAMState.h"
#include "../visualisation/VisualisationGenerator.h"

namespace spaint {

/**
 * \brief An instance of a class deriving from this one provides the shared context needed by a collaborative component.
 */
class CollaborativeContext
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The pose graph optimiser. */
  PoseGraphOptimiser_Ptr m_poseGraphOptimiser;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a collaborative context.
   */
  CollaborativeContext();

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  virtual itmx::RefiningRelocaliser_CPtr get_relocaliser(const std::string& sceneID) const = 0;
  virtual std::vector<std::string> get_scene_ids() const = 0;
  virtual SLAMState_CPtr get_slam_state(const std::string& sceneID) const = 0;
  virtual VisualisationGenerator_CPtr get_visualisation_generator() const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  virtual const PoseGraphOptimiser_Ptr& get_pose_graph_optimiser();
  virtual PoseGraphOptimiser_CPtr get_pose_graph_optimiser() const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<CollaborativeContext> CollaborativeContext_Ptr;

}

#endif
