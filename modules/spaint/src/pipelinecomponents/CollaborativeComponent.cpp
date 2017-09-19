/**
 * spaint: CollaborativeComponent.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "pipelinecomponents/CollaborativeComponent.h"
using namespace ORUtils;

#include <algorithm>

#include <boost/bind.hpp>
using boost::bind;

#include <itmx/relocalisation/Relocaliser.h>
using namespace itmx;

#include "collaboration/SubmapRelocalisation.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

CollaborativeComponent::CollaborativeComponent(const CollaborativeContext_Ptr& context)
: m_context(context), m_frameIndex(0)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void CollaborativeComponent::run_collaborative_pose_estimation()
{
  const float failurePenaltyIncrease = 1.0f;
  const float failurePenaltyMax = 5.0f;
  const int maxRelocalisationsNeeded = 3;

  const std::vector<std::string> sceneIDs = m_context->get_scene_ids();
  const int sceneCount = static_cast<int>(sceneIDs.size());

  typedef boost::shared_ptr<SubmapRelocalisation> SubmapRelocalisation_Ptr;
  typedef std::pair<SubmapRelocalisation_Ptr,float> Candidate;
  static std::list<Candidate> candidates;
  static std::list<Candidate> redundantCandidates;

  if(m_frameIndex > 0)
  {
    // TODO: Comment here.
    update_failure_penalties();

    if(m_frameIndex % 50 == 0)
    {
      std::vector<std::pair<ITMFloatImage_Ptr,ITMUChar4Image_Ptr> > rgbdImages;
      for(size_t j = 0; j < sceneCount; ++j)
      {
        const SLAMState_CPtr slamStateJ = m_context->get_slam_state(sceneIDs[j]);
        const View_CPtr viewJ = slamStateJ->get_view();

        viewJ->depth->UpdateHostFromDevice();
        viewJ->rgb->UpdateHostFromDevice();

        ITMFloatImage_Ptr depthJ(new ITMFloatImage(viewJ->depth->noDims, true, false));
        ITMUChar4Image_Ptr rgbJ(new ITMUChar4Image(viewJ->rgb->noDims, true, false));
        depthJ->SetFrom(viewJ->depth, ITMFloatImage::CPU_TO_CPU);
        rgbJ->SetFrom(viewJ->rgb, ITMUChar4Image::CPU_TO_CPU);

        rgbdImages.push_back(std::make_pair(depthJ, rgbJ));
      }

      for(size_t i = 0; i < sceneCount; ++i)
      {
        for(size_t j = 0; j < sceneCount; ++j)
        {
          if(j == i) continue;

          const std::string sceneI = sceneIDs[i];
          const std::string sceneJ = sceneIDs[j];

          boost::optional<CollaborativeContext::SE3PoseCluster> largestCluster = m_context->try_get_largest_cluster(sceneI, sceneJ);
          const bool redundant = largestCluster && largestCluster->size() >= maxRelocalisationsNeeded;

          if(!redundant)
          {
            const SLAMState_CPtr slamStateJ = m_context->get_slam_state(sceneJ);
            SubmapRelocalisation_Ptr candidate(
              new SubmapRelocalisation(sceneI, sceneJ, m_frameIndex, rgbdImages[j].first, rgbdImages[j].second, slamStateJ->get_intrinsics().projectionParamsSimple.all, slamStateJ->get_pose())
            );
            (redundant ? redundantCandidates : candidates).push_back(std::make_pair(candidate, 0.0f));
          }
        }
      }
    }

    if(m_frameIndex % 100 == 0)
    {
      for(std::list<Candidate>::iterator it = candidates.begin(), iend = candidates.end(); it != iend; ++it)
      {
        SubmapRelocalisation_Ptr candidate = it->first;
        boost::optional<CollaborativeContext::SE3PoseCluster> largestCluster = m_context->try_get_largest_cluster(candidate->m_sceneI, candidate->m_sceneJ);
        size_t largestClusterSize = largestCluster ? largestCluster->size() : 0;
        float sizeDiff = static_cast<float>(largestClusterSize) - maxRelocalisationsNeeded / 2.0f;
        //float sizeTerm = sizeDiff * sizeDiff;
        float sizeTerm = maxRelocalisationsNeeded - static_cast<float>(largestClusterSize);
        float primaryTerm = candidate->m_sceneI == "World" || candidate->m_sceneJ == "World" ? 5.0f : 0.0f;
        float score = sizeTerm + primaryTerm - m_failurePenalties[std::make_pair(candidate->m_sceneI, candidate->m_sceneJ)];
        it->second = score;
      }

      candidates.sort(bind(&Candidate::second, _1) < bind(&Candidate::second, _2));

      std::cout << "BEGIN CANDIDATES " << m_frameIndex << '\n';
      for(std::list<Candidate>::const_iterator it = candidates.begin(), iend = candidates.end(); it != iend; ++it)
      {
        const SubmapRelocalisation_Ptr& candidate = it->first;
        const float score = it->second;
        std::cout << candidate->m_sceneI << ' ' << candidate->m_sceneJ << ' ' << candidate->m_frameIndex << ' ' << score << '\n';
      }
      std::cout << "END CANDIDATES\n";

      // Try to relocalise the best candidate (if any).
      if(!candidates.empty())
      {
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
          boost::optional<CollaborativeContext::SE3PoseCluster> largestCluster = m_context->try_get_largest_cluster(bestCandidate->m_sceneI, bestCandidate->m_sceneJ);
          if(largestCluster && largestCluster->size() >= maxRelocalisationsNeeded)
          {
            for(std::list<Candidate>::iterator it = candidates.begin(), iend = candidates.end(); it != iend;)
            {
              if((it->first->m_sceneI == bestCandidate->m_sceneI && it->first->m_sceneJ == bestCandidate->m_sceneJ) ||
                 (it->first->m_sceneI == bestCandidate->m_sceneJ && it->first->m_sceneI == bestCandidate->m_sceneJ))
              {
                redundantCandidates.push_back(*it);
                it = candidates.erase(it);
              }
              else ++it;
            }
          }
        }
        else
        {
          std::cout << "failed :(\n";
          float& failurePenalty = m_failurePenalties[std::make_pair(bestCandidate->m_sceneI, bestCandidate->m_sceneJ)];
          failurePenalty = std::min(failurePenalty + failurePenaltyIncrease, failurePenaltyMax);
        }
      }
    }
  }
  ++m_frameIndex;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void CollaborativeComponent::update_failure_penalties()
{
  const float failurePenaltyDecreasePerFrame = 0.001f;
  const std::vector<std::string> sceneIDs = m_context->get_scene_ids();

  for(size_t i = 0, sceneCount = sceneIDs.size(); i < sceneCount; ++i)
  {
    for(size_t j = 0; j < sceneCount; ++j)
    {
      float& failurePenalty = m_failurePenalties[std::make_pair(sceneIDs[i], sceneIDs[j])];
      failurePenalty = std::max(failurePenalty - failurePenaltyDecreasePerFrame, 0.0f);
    }
  }
}

}
