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

namespace spaint {

//#################### CONSTRUCTORS ####################

CollaborativeComponent::CollaborativeComponent(const CollaborativeContext_Ptr& context)
: m_context(context),
  m_failurePenaltyDecreasePerFrame(0.001f),
  m_frameIndex(0),
  m_maxRelocalisationsNeeded(3),
  m_stopRelocalisationThread(false)
{}

//#################### DESTRUCTOR ####################

CollaborativeComponent::~CollaborativeComponent()
{
  m_stopRelocalisationThread = true;
  m_readyToRelocalise.notify_one();
  m_relocalisationThread.join();
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void CollaborativeComponent::run_collaborative_pose_estimation()
{
  const float failurePenaltyIncrease = 1.0f;
  const float failurePenaltyMax = 5.0f;
  const std::vector<std::string> sceneIDs = m_context->get_scene_ids();
  const int sceneCount = static_cast<int>(sceneIDs.size());

  // Start the relocalisation thread if it isn't already running.
  static bool threadStarted = false;
  if(!threadStarted)
  {
    m_relocalisationThread = boost::thread(boost::bind(&CollaborativeComponent::run_relocalisation, this));
    threadStarted = true;
  }

  if(m_frameIndex > 0)
  {
    // TODO: Comment here.
    update_failure_penalties();

    // TODO: Comment here.
    if(m_frameIndex % 50 == 0) add_relocalisation_candidates();

    // TODO: Comment here.
    if(m_frameIndex % 100 == 0 && !m_candidates.empty())
    {
      // TODO: Comment here.
      score_relocalisation_candidates();

      bool readyToRelocalise = false;

      {
        boost::unique_lock<boost::mutex> lock(m_mutex);
        if(!m_bestCandidate)
        {
          // Try to relocalise the best candidate.
          m_bestCandidate = m_candidates.back().first;
          m_candidates.pop_back();
          readyToRelocalise = true;
        }
      }

      if(readyToRelocalise)
      {
        std::cout << "Signalling relocalisation thread" << std::endl;
        m_readyToRelocalise.notify_one();
      }

#if 0
      std::cout << "Attempting to relocalise " << m_bestCandidate->m_sceneJ << " against " << m_bestCandidate->m_sceneI << "...";
      Relocaliser_CPtr relocaliserI = m_context->get_relocaliser(m_bestCandidate->m_sceneI);

      const SLAMState_CPtr slamStateJ = m_context->get_slam_state(m_bestCandidate->m_sceneJ);
      ITMFloatImage_Ptr depth(new ITMFloatImage(slamStateJ->get_depth_image_size(), true, true));
      ITMUChar4Image_Ptr rgb(new ITMUChar4Image(slamStateJ->get_rgb_image_size(), true, true));
      depth->SetFrom(m_bestCandidate->m_depthJ.get(), ITMFloatImage::CPU_TO_CPU);
      rgb->SetFrom(m_bestCandidate->m_rgbJ.get(), ITMUChar4Image::CPU_TO_CPU);
      depth->UpdateDeviceFromHost();
      rgb->UpdateDeviceFromHost();

      boost::optional<Relocaliser::Result> result = relocaliserI->relocalise(rgb.get(), depth.get(), m_bestCandidate->m_depthIntrinsicsJ);
      if(result && result->quality == Relocaliser::RELOCALISATION_GOOD)
      {
        // cjTwi^-1 * cjTwj = wiTcj * cjTwj = wiTwj
        m_bestCandidate->m_relativePose = ORUtils::SE3Pose(result->pose.GetInvM() * m_bestCandidate->m_localPoseJ.GetM());
        std::cout << "succeeded!\n";
        //std::cout << bestCandidate->m_relativePose->GetM() << '\n';
        m_context->add_relative_transform_sample(m_bestCandidate->m_sceneI, m_bestCandidate->m_sceneJ, *m_bestCandidate->m_relativePose); // TEMPORARY

        // If we've now got enough relocalisations for this pair of scenes, move all the remaining candidates for them to a separate list.
        boost::optional<CollaborativeContext::SE3PoseCluster> largestCluster = m_context->try_get_largest_cluster(m_bestCandidate->m_sceneI, m_bestCandidate->m_sceneJ);
        if(largestCluster && largestCluster->size() >= m_maxRelocalisationsNeeded)
        {
          for(std::list<Candidate>::iterator it = m_candidates.begin(), iend = m_candidates.end(); it != iend;)
          {
            if((it->first->m_sceneI == m_bestCandidate->m_sceneI && it->first->m_sceneJ == m_bestCandidate->m_sceneJ) ||
                (it->first->m_sceneI == m_bestCandidate->m_sceneJ && it->first->m_sceneI == m_bestCandidate->m_sceneJ))
            {
              m_redundantCandidates.push_back(*it);
              it = m_candidates.erase(it);
            }
            else ++it;
          }
        }
      }
      else
      {
        std::cout << "failed :(\n";
        float& failurePenalty = m_failurePenalties[std::make_pair(m_bestCandidate->m_sceneI, m_bestCandidate->m_sceneJ)];
        failurePenalty = std::min(failurePenalty + failurePenaltyIncrease, failurePenaltyMax);
      }

      m_bestCandidate.reset();
#endif
    }
  }
  ++m_frameIndex;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void CollaborativeComponent::add_relocalisation_candidates()
{
  const std::vector<std::string> sceneIDs = m_context->get_scene_ids();
  const int sceneCount = static_cast<int>(sceneIDs.size());

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
      const bool redundant = largestCluster && largestCluster->size() >= m_maxRelocalisationsNeeded;

      if(!redundant)
      {
        const SLAMState_CPtr slamStateJ = m_context->get_slam_state(sceneJ);
        SubmapRelocalisation_Ptr candidate(
          new SubmapRelocalisation(sceneI, sceneJ, m_frameIndex, rgbdImages[j].first, rgbdImages[j].second, slamStateJ->get_intrinsics().projectionParamsSimple.all, slamStateJ->get_pose())
        );
        (redundant ? m_redundantCandidates : m_candidates).push_back(std::make_pair(candidate, 0.0f));
      }
    }
  }
}

void CollaborativeComponent::run_relocalisation()
{
  while(!m_stopRelocalisationThread)
  {
    {
      boost::unique_lock<boost::mutex> lock(m_mutex);
      while(!m_bestCandidate)
      {
        m_readyToRelocalise.wait(lock);

        // TODO: Comment here.
        if(m_stopRelocalisationThread) return;
      }
    }

    std::cout << "Running relocalisation " << m_frameIndex << '\n';
    // TODO
    std::cout << "Relocalisation finished\n";

    {
      boost::unique_lock<boost::mutex> lock(m_mutex);
      m_bestCandidate.reset();
    }
  }
}

void CollaborativeComponent::schedule_relocalisation()
{
  // TODO
}

void CollaborativeComponent::score_relocalisation_candidates()
{
  for(std::list<Candidate>::iterator it = m_candidates.begin(), iend = m_candidates.end(); it != iend; ++it)
  {
    SubmapRelocalisation_Ptr candidate = it->first;
    boost::optional<CollaborativeContext::SE3PoseCluster> largestCluster = m_context->try_get_largest_cluster(candidate->m_sceneI, candidate->m_sceneJ);
    size_t largestClusterSize = largestCluster ? largestCluster->size() : 0;
    float sizeDiff = static_cast<float>(largestClusterSize) - m_maxRelocalisationsNeeded / 2.0f;
    //float sizeTerm = sizeDiff * sizeDiff;
    float sizeTerm = m_maxRelocalisationsNeeded - static_cast<float>(largestClusterSize);
    float primaryTerm = candidate->m_sceneI == "World" || candidate->m_sceneJ == "World" ? 5.0f : 0.0f;
    float score = sizeTerm + primaryTerm - m_failurePenalties[std::make_pair(candidate->m_sceneI, candidate->m_sceneJ)];
    it->second = score;
  }

  m_candidates.sort(bind(&Candidate::second, _1) < bind(&Candidate::second, _2));

#if 0
  std::cout << "BEGIN CANDIDATES " << m_frameIndex << '\n';
  for(std::list<Candidate>::const_iterator it = m_candidates.begin(), iend = m_candidates.end(); it != iend; ++it)
  {
    const SubmapRelocalisation_Ptr& candidate = it->first;
    const float score = it->second;
    std::cout << candidate->m_sceneI << ' ' << candidate->m_sceneJ << ' ' << candidate->m_frameIndex << ' ' << score << '\n';
  }
  std::cout << "END CANDIDATES\n";
#endif
}

void CollaborativeComponent::update_failure_penalties()
{
  const std::vector<std::string> sceneIDs = m_context->get_scene_ids();
  for(size_t i = 0, sceneCount = sceneIDs.size(); i < sceneCount; ++i)
  {
    for(size_t j = 0; j < sceneCount; ++j)
    {
      float& failurePenalty = m_failurePenalties[std::make_pair(sceneIDs[i], sceneIDs[j])];
      failurePenalty = std::max(failurePenalty - m_failurePenaltyDecreasePerFrame, 0.0f);
    }
  }
}

}
