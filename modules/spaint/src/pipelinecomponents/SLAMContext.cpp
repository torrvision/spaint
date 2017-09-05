/**
 * spaint: SLAMContext.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/SLAMContext.h"

#include <tvgutil/containers/MapUtil.h>
using namespace tvgutil;

namespace spaint {

//#################### PUBLIC MEMBER FUNCTIONS ####################

itmx::RefiningRelocaliser_Ptr& SLAMContext::get_relocaliser(const std::string& sceneID)
{
  return m_relocalisers[sceneID];
}

itmx::RefiningRelocaliser_CPtr SLAMContext::get_relocaliser(const std::string& sceneID) const
{
  return MapUtil::lookup(m_relocalisers, sceneID);
}

std::vector<std::string> SLAMContext::get_scene_ids() const
{
  std::vector<std::string> result;
  result.reserve(m_slamStates.size());
  for(std::map<std::string,SLAMState_Ptr>::const_iterator it = m_slamStates.begin(), iend = m_slamStates.end(); it != iend; ++it)
  {
    result.push_back(it->first);
  }
  return result;
}

const SLAMState_Ptr& SLAMContext::get_slam_state(const std::string& sceneID)
{
  SLAMState_Ptr& result = m_slamStates[sceneID];
  if(!result) result.reset(new SLAMState);
  return result;
}

SLAMState_CPtr SLAMContext::get_slam_state(const std::string& sceneID) const
{
  return MapUtil::lookup(m_slamStates, sceneID);
}

}
