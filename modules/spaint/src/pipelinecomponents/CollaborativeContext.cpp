/**
 * spaint: CollaborativeContext.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "pipelinecomponents/CollaborativeContext.h"

#include <iostream>

#include <itmx/geometry/GeometryUtil.h>
using namespace itmx;

namespace spaint {

//#################### DESTRUCTOR ####################

CollaborativeContext::~CollaborativeContext() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

boost::optional<ORUtils::SE3Pose> CollaborativeContext::try_get_relative_transform(const std::string& sceneI, const std::string& sceneJ) const
{
  //std::cout << "Looking for samples: " << sceneI << "<-" << sceneJ << "...";

  // Try to look up the samples of the relative transformation from the coordinate system of scene j to that of scene i.
  std::map<std::pair<std::string,std::string>,std::vector<ORUtils::SE3Pose> >::const_iterator it = m_relativeTransformSamples.find(std::make_pair(sceneI, sceneJ));

  // If there aren't any, it's because we haven't found the relative transformation between the two scenes yet, so early out.
  if(it == m_relativeTransformSamples.end())
  {
    //std::cout << "failed\n";
    return boost::none;
  }

  //std::cout << "succeeded\n";

  // Otherwise, blend the samples together to get a better estimate of the relative transformation and return the result.
  return GeometryUtil::blend_poses(it->second);
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void CollaborativeContext::add_relative_transform_sample(const std::string& sceneI, const std::string& sceneJ, const ORUtils::SE3Pose& sample)
{
  std::cout << "Adding sample: " << sceneI << "<-" << sceneJ << '\n' << sample.GetM() << '\n';
  m_relativeTransformSamples[std::make_pair(sceneI, sceneJ)].push_back(sample);

  std::cout << "Adding sample: " << sceneJ << "<-" << sceneI << '\n' << sample.GetInvM() << '\n';
  m_relativeTransformSamples[std::make_pair(sceneJ, sceneI)].push_back(ORUtils::SE3Pose(sample.GetInvM()));
}

}
