/**
 * spaint: SLAMContext.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/SLAMContext.h"

#include <tvgutil/containers/MapUtil.h>
using namespace tvgutil;

namespace spaint {

//#################### PUBLIC MEMBER FUNCTIONS ####################

void SLAMContext::add_scene_id(const std::string& sceneID)
{
  m_sceneIDs.push_back(sceneID);
}

itmx::RefiningRelocaliser_Ptr& SLAMContext::get_relocaliser(const std::string& sceneID)
{
  return m_relocalisers[sceneID];
}

itmx::RefiningRelocaliser_CPtr SLAMContext::get_relocaliser(const std::string& sceneID) const
{
  return MapUtil::lookup(m_relocalisers, sceneID);
}

const std::vector<std::string>& SLAMContext::get_scene_ids() const
{
  return m_sceneIDs;
}

const SLAMState_Ptr& SLAMContext::get_slam_state(const std::string& sceneID)
{
  SLAMState_Ptr& result = m_slamStates[sceneID];
  if(!result) result.reset(new SLAMState);
  return result;
}

SLAMState_CPtr SLAMContext::get_slam_state(const std::string& sceneID) const
{
  std::map<std::string,SLAMState_Ptr>::const_iterator it = m_slamStates.find(sceneID);
  return it != m_slamStates.end() ? it->second : SLAMState_CPtr();
}

}
