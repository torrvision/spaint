/**
 * spaint: CollaborativeContext.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "pipelinecomponents/CollaborativeContext.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

CollaborativeContext::CollaborativeContext()
: m_poseGraphOptimiser(new PoseGraphOptimiser)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

const PoseGraphOptimiser_Ptr& CollaborativeContext::get_pose_graph_optimiser()
{
  return m_poseGraphOptimiser;
}

PoseGraphOptimiser_CPtr CollaborativeContext::get_pose_graph_optimiser() const
{
  return m_poseGraphOptimiser;
}

}
