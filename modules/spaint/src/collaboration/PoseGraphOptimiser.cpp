/**
 * spaint: PoseGraphOptimiser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "collaboration/PoseGraphOptimiser.h"
using namespace ORUtils;

#include <MiniSlamGraphLib/GraphEdgeSE3.h>
#include <MiniSlamGraphLib/GraphNodeSE3.h>
#include <MiniSlamGraphLib/LevenbergMarquardtMethod.h>
#include <MiniSlamGraphLib/PoseGraph.h>
#include <MiniSlamGraphLib/SlamGraphErrorFunction.h>
using namespace MiniSlamGraph;

#include <itmx/geometry/GeometryUtil.h>
using namespace itmx;

namespace spaint {

//#################### CONSTRUCTORS ####################

PoseGraphOptimiser::PoseGraphOptimiser()
: m_optimisationThread(new boost::thread(boost::bind(&PoseGraphOptimiser::run_pose_graph_optimisation, this))),
  m_relativeTransformSamplesChanged(false),
  m_shouldTerminate(false)
{}

//#################### DESTRUCTOR ####################

PoseGraphOptimiser::~PoseGraphOptimiser()
{
  m_shouldTerminate = true;

  if(m_optimisationThread)
  {
    // Artificially wake up the pose graph optimisation thread and wait for it to terminate.
    m_relativeTransformSamplesAdded.notify_one();
    m_optimisationThread->join();
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void PoseGraphOptimiser::add_relative_transform_sample(const std::string& sceneI, const std::string& sceneJ, const SE3Pose& sample)
{
  boost::lock_guard<boost::mutex> lock(m_mutex);

  add_relative_transform_sample_sub(sceneI, sceneJ, sample);
  add_relative_transform_sample_sub(sceneJ, sceneI, SE3Pose(sample.GetInvM()));

  m_relativeTransformSamplesChanged = true;
  m_relativeTransformSamplesAdded.notify_one();
}

boost::optional<PoseGraphOptimiser::SE3PoseCluster>
PoseGraphOptimiser::try_get_largest_cluster(const std::string& sceneI, const std::string& sceneJ) const
{
  boost::lock_guard<boost::mutex> lock(m_mutex);

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

boost::optional<std::pair<SE3Pose,size_t> > PoseGraphOptimiser::try_get_relative_transform(const std::string& sceneI, const std::string& sceneJ) const
{
  boost::optional<SE3PoseCluster> largestCluster = try_get_largest_cluster(sceneI, sceneJ);
  return largestCluster ? boost::optional<std::pair<SE3Pose,size_t> >(std::make_pair(GeometryUtil::blend_poses(*largestCluster), largestCluster->size())) : boost::none;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void PoseGraphOptimiser::add_relative_transform_sample_sub(const std::string& sceneI, const std::string& sceneJ, const SE3Pose& sample)
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

void PoseGraphOptimiser::run_pose_graph_optimisation()
{
  std::cout << "Starting pose graph optimisation thread" << std::endl;

  while(!m_shouldTerminate)
  {
    PoseGraph graph;

    {
      boost::unique_lock<boost::mutex> lock(m_mutex);

      // Wait for samples to be added or the termination flag to be set.
      while(!m_relativeTransformSamplesChanged && !m_shouldTerminate) m_relativeTransformSamplesAdded.wait(lock);

      // If the termination flag is set, early out.
      if(m_shouldTerminate) return;

      // Reset the change flag.
      m_relativeTransformSamplesChanged = false;

      // Construct the pose graph.
      // TODO
    }

    // Run the pose graph optimisation.
    graph.prepareEvaluations();
    SlamGraphErrorFunction errFunc(graph);
    SlamGraphErrorFunction::Parameters params(graph);
    LevenbergMarquardtMethod::minimize(errFunc, params);
    graph.setNodeIndex(params.getNodes());

    {
      boost::lock_guard<boost::mutex> lock(m_mutex);

      // Extract and store the optimised poses.
      // TODO
    }
  }
}

}
