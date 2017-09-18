/**
 * spaint: CollaborativeComponent.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "pipelinecomponents/CollaborativeComponent.h"
using namespace ORUtils;

#include <algorithm>

#include <boost/bind.hpp>
using boost::bind;

#include <itmx/geometry/GeometryUtil.h>
#include <itmx/relocalisation/Relocaliser.h>
using namespace itmx;

#include <tvgutil/containers/PriorityQueue.h>
#include <tvgutil/numbers/RandomNumberGenerator.h>
using namespace tvgutil;

#include "collaboration/SubmapRelocalisation.h"

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

#if 1
  typedef boost::shared_ptr<SubmapRelocalisation> SubmapRelocalisation_Ptr;
  typedef std::pair<SubmapRelocalisation_Ptr,float> Candidate;
  static std::vector<Candidate> candidates;

  const int maxRelocalisationsNeeded = 3;

  static int frameIndex = 0;
  if(frameIndex > 0)
  {
    if(frameIndex % 50 == 0)
    {
      for(size_t i = 0; i < sceneCount; ++i)
      {
        for(size_t j = 0; j < sceneCount; ++j)
        {
          if(j == i) continue;

          const std::string sceneI = sceneIDs[i];
          const std::string sceneJ = sceneIDs[j];
          const SLAMState_CPtr slamStateJ = m_context->get_slam_state(sceneJ);
          SubmapRelocalisation_Ptr candidate(new SubmapRelocalisation(sceneI, sceneJ, frameIndex, slamStateJ->get_view(), slamStateJ->get_pose()));
          candidates.push_back(std::make_pair(candidate, 0.0f));
        }
      }
    }

    if(frameIndex % 100 == 0)
    {
      for(size_t i = 0, candidateCount = candidates.size(); i < candidateCount; ++i)
      {
        SubmapRelocalisation_Ptr candidate = candidates[i].first;
        boost::optional<CollaborativeContext::SE3PoseCluster> largestCluster = m_context->try_get_largest_cluster(candidate->m_sceneI, candidate->m_sceneJ);
        size_t largestClusterSize = largestCluster ? largestCluster->size() : 0;
        float sizeDiff = static_cast<float>(largestClusterSize) - maxRelocalisationsNeeded / 2.0f;
        float score = sizeDiff * sizeDiff;
        candidates[i].second = score;
      }

      std::sort(candidates.begin(), candidates.end(), bind(&Candidate::second, _1) < bind(&Candidate::second, _2));

      std::cout << "BEGIN CANDIDATES " << frameIndex << '\n';
      for(size_t i = 0, candidateCount = candidates.size(); i < candidateCount; ++i)
      {
        const SubmapRelocalisation_Ptr& candidate = candidates[i].first;
        const float score = candidates[i].second;
        std::cout << candidate->m_sceneI << ' ' << candidate->m_sceneJ << ' ' << candidate->m_frameIndex << ' ' << score << '\n';
      }
      std::cout << "END CANDIDATES\n";

      // Try to relocalise the best candidate.
      SubmapRelocalisation_Ptr bestCandidate = candidates.back().first;
      candidates.pop_back();

      std::cout << "Attempting to relocalise " << bestCandidate->m_sceneJ << " against " << bestCandidate->m_sceneI << "...";
      Relocaliser_CPtr relocaliserI = m_context->get_relocaliser(bestCandidate->m_sceneI);

      const SLAMState_CPtr slamStateJ = m_context->get_slam_state(bestCandidate->m_sceneJ);
      ITMFloatImage_Ptr depth(new ITMFloatImage(slamStateJ->get_depth_image_size(), true, true));
      ITMUChar4Image_Ptr rgb(new ITMUChar4Image(slamStateJ->get_rgb_image_size(), true, true));
      depth->SetFrom(bestCandidate->m_depthJ.get(), ITMFloatImage::CPU_TO_CPU);
      rgb->SetFrom(bestCandidate->m_rgbJ.get(), ITMUChar4Image::CPU_TO_CPU);
      depth->UpdateDeviceFromHost();
      rgb->UpdateDeviceFromHost();

      boost::optional<Relocaliser::Result> result = relocaliserI->relocalise(rgb.get(), depth.get(), bestCandidate->m_depthIntrinsicsJ);
      if(result && result->quality == Relocaliser::RELOCALISATION_GOOD)
      {
        // cjTwi^-1 * cjTwj = wiTcj * cjTwj = wiTwj
        bestCandidate->m_relativePose = ORUtils::SE3Pose(result->pose.GetInvM() * bestCandidate->m_localPoseJ.GetM());
        std::cout << "succeeded!\n";
        //std::cout << bestCandidate->m_relativePose->GetM() << '\n';
        m_context->add_relative_transform_sample(bestCandidate->m_sceneI, bestCandidate->m_sceneJ, *bestCandidate->m_relativePose); // TEMPORARY

        // If we've now got enough relocalisations for this pair of scenes, move all the remaining candidates for them to a separate list.
        // TODO
      }
      else std::cout << "failed :(\n";
    }
  }
  ++frameIndex;
#else
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
#endif
}

}
