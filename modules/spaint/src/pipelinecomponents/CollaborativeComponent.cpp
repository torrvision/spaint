/**
 * spaint: CollaborativeComponent.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "pipelinecomponents/CollaborativeComponent.h"
using namespace ORUtils;

#include <itmx/geometry/GeometryUtil.h>
#include <itmx/relocalisation/Relocaliser.h>
using namespace itmx;

#include <tvgutil/numbers/RandomNumberGenerator.h>
using namespace tvgutil;

namespace spaint {

//#################### CONSTRUCTORS ####################

CollaborativeComponent::CollaborativeComponent(const CollaborativeContext_Ptr& context)
: m_context(context)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void CollaborativeComponent::run_collaborative_pose_estimation()
{
  std::vector<std::string> sceneIDs = m_context->get_scene_ids();
  int sceneCount = static_cast<int>(sceneIDs.size());

  static RandomNumberGenerator rng(12345);
  static std::deque<CollaborativeContext::SceneIDPair> recentSceneIDPairs;

  static int count = 0;
  if(count > 0 && count % 100 == 0)
  {
    // Pick a pair of scenes to relocalise with respect to each other.
    std::map<int,std::vector<CollaborativeContext::SceneIDPair> > largestClusterSizeToSceneIDPairs;
    std::map<int,std::vector<CollaborativeContext::SceneIDPair> > filteredLargestClusterSizeToSceneIDPairs;
    for(size_t i = 0; i < sceneCount; ++i)
    {
      for(size_t j = 0; j < sceneCount; ++j)
      {
        if(j == i) continue;
        boost::optional<CollaborativeContext::SE3PoseCluster> largestCluster = m_context->try_get_largest_cluster(sceneIDs[i], sceneIDs[j]);
        int largestClusterSize = largestCluster ? (int)largestCluster->size() : 0;
        const std::pair<std::string,std::string> sceneIDs(sceneIDs[i], sceneIDs[j]);
        largestClusterSizeToSceneIDPairs[largestClusterSize].push_back(sceneIDs);
        if(std::find(recentSceneIDPairs.begin(), recentSceneIDPairs.end(), sceneIDs) == recentSceneIDPairs.end())
        {
          filteredLargestClusterSizeToSceneIDPairs[largestClusterSize].push_back(sceneIDs);
        }
      }
    }

    if(largestClusterSizeToSceneIDPairs.begin()->first >= 3) return;
    std::map<int,std::vector<CollaborativeContext::SceneIDPair> >& m = filteredLargestClusterSizeToSceneIDPairs.empty() ? largestClusterSizeToSceneIDPairs : filteredLargestClusterSizeToSceneIDPairs;
    const std::vector<CollaborativeContext::SceneIDPair>& candidates = m.begin()->second;
    int k = rng.generate_int_from_uniform(0, static_cast<int>(candidates.size()) - 1);
    const std::string sceneI = candidates[k].first;
    const std::string sceneJ = candidates[k].second;

    // TODO: Comment here.
    recentSceneIDPairs.push_back(std::make_pair(sceneI, sceneJ));
    if(recentSceneIDPairs.size() > 5) recentSceneIDPairs.pop_front();

    // Try to relocalise the current frame of scene j against scene i.
    Relocaliser_CPtr relocaliserI = m_context->get_relocaliser(sceneI);
    View_CPtr viewJ = m_context->get_slam_state(sceneJ)->get_view();
    SE3Pose localPoseJ = m_context->get_slam_state(sceneJ)->get_pose();

    std::cout << "Attempting to relocalise " << sceneJ << " against " << sceneI << "...";
    boost::optional<Relocaliser::Result> result = relocaliserI->relocalise(viewJ->rgb, viewJ->depth, viewJ->calib.intrinsics_d.projectionParamsSimple.all);
    if(result && result->quality == Relocaliser::RELOCALISATION_GOOD)
    {
      // cjTwi^-1 * cjTwj = wiTcj * cjTwj = wiTwj
      SE3Pose relativePose = ORUtils::SE3Pose(result->pose.GetInvM() * localPoseJ.GetM());
      std::cout << "succeeded!\n";
      //std::cout << relativePose.GetM() << '\n';
      m_context->add_relative_transform_sample(sceneI, sceneJ, relativePose);
    }
    else std::cout << "failed :(\n";
  }
  ++count;
}

}
