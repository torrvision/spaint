/**
 * spaint: CollaborativeContext.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "pipelinecomponents/CollaborativeContext.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

CollaborativeContext::CollaborativeContext()
: m_collaborativePoseOptimiser(new CollaborativePoseOptimiser)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

const CollaborativePoseOptimiser_Ptr& CollaborativeContext::get_collaborative_pose_optimiser()
{
  return m_collaborativePoseOptimiser;
}

CollaborativePoseOptimiser_CPtr CollaborativeContext::get_collaborative_pose_optimiser() const
{
  return m_collaborativePoseOptimiser;
}

}
