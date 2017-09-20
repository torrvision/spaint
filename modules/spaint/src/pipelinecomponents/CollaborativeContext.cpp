/**
 * spaint: CollaborativeContext.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "pipelinecomponents/CollaborativeContext.h"
using namespace ORUtils;

#include <itmx/geometry/GeometryUtil.h>
using namespace itmx;

namespace spaint {

//#################### DESTRUCTOR ####################

CollaborativeContext::~CollaborativeContext() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

boost::optional<std::pair<SE3Pose,size_t> > CollaborativeContext::try_get_relative_transform(const std::string& sceneI, const std::string& sceneJ) const
{
  boost::lock_guard<boost::recursive_mutex> lock(m_mutex);

  boost::optional<SE3PoseCluster> largestCluster = try_get_largest_cluster(sceneI, sceneJ);
  return largestCluster ? boost::optional<std::pair<SE3Pose,size_t> >(std::make_pair(GeometryUtil::blend_poses(*largestCluster), largestCluster->size())) : boost::none;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void CollaborativeContext::add_relative_transform_sample(const std::string& sceneI, const std::string& sceneJ, const SE3Pose& sample)
{
  boost::lock_guard<boost::recursive_mutex> lock(m_mutex);

  add_relative_transform_sample_sub(sceneI, sceneJ, sample);
  add_relative_transform_sample_sub(sceneJ, sceneI, SE3Pose(sample.GetInvM()));
}

void CollaborativeContext::add_relative_transform_sample_sub(const std::string& sceneI, const std::string& sceneJ, const SE3Pose& sample)
{
  std::cout << "Adding sample: " << sceneI << "<-" << sceneJ << '\n' << sample.GetM() << '\n';

  // Try to find an existing cluster that contains a sample that is sufficiently similar to the new sample.
  // If we find one, add the sample to that cluster and early out.
  std::vector<SE3PoseCluster>& clusters = m_relativeTransformSamples[std::make_pair(sceneI, sceneJ)];
  for(size_t i = 0, clusterCount = clusters.size(); i < clusterCount; ++i)
  {
    for(size_t j = 0, size = clusters[i].size(); j < size; ++j)
    {
      if(GeometryUtil::poses_are_similar(sample, clusters[i][j]))
      {
        clusters[i].push_back(sample);
        return;
      }
    }
  }

  // If the new sample is not sufficiently similar to the samples in any of the existing clusters, create a new cluster for it.
  SE3PoseCluster newCluster;
  newCluster.push_back(sample);
  clusters.push_back(newCluster);
}

boost::optional<CollaborativeContext::SE3PoseCluster>
CollaborativeContext::try_get_largest_cluster(const std::string& sceneI, const std::string& sceneJ) const
{
  boost::lock_guard<boost::recursive_mutex> lock(m_mutex);

  // Try to look up the sample clusters of the relative transformation from the coordinate system of scene j to that of scene i.
  std::map<SceneIDPair,std::vector<SE3PoseCluster> >::const_iterator it = m_relativeTransformSamples.find(std::make_pair(sceneI, sceneJ));

  // If there aren't any, it's because we haven't found the relative transformation between the two scenes yet, so early out.
  if(it == m_relativeTransformSamples.end()) return boost::none;

  // Otherwise, find a largest cluster and return it.
  const std::vector<SE3PoseCluster>& clusters = it->second;
  const SE3PoseCluster *largestCluster = NULL;
  size_t largestClusterSize = 0;
  for(size_t i = 0, clusterCount = clusters.size(); i < clusterCount; ++i)
  {
    size_t clusterSize = clusters[i].size();
    if(clusterSize > largestClusterSize)
    {
      largestCluster = &clusters[i];
      largestClusterSize = clusterSize;
    }
  }

  return largestCluster ? boost::optional<SE3PoseCluster>(*largestCluster) : boost::none;
}

}
