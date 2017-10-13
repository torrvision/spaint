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
#ifdef WITH_OPENCV
#include <itmx/ocv/OpenCVUtil.h>
#endif
#include <itmx/relocalisation/Relocaliser.h>
using namespace itmx;

#define USE_REAL_IMAGES 0
#define SAVE_REAL_IMAGES USE_REAL_IMAGES

namespace spaint {

//#################### CONSTRUCTORS ####################

CollaborativeComponent::CollaborativeComponent(const CollaborativeContext_Ptr& context)
: m_context(context),
  m_frameIndex(0),
  m_maxRelocalisationsNeeded(5),
  m_stopRelocalisationThread(false)
{
  m_relocalisationThread = boost::thread(boost::bind(&CollaborativeComponent::run_relocalisation, this));
}

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
  if(m_frameIndex > 0)
  {
    // TODO: Comment here.
    update_failure_penalties();

    // TODO: Comment here.
    add_relocalisation_candidates();

    // TODO: Comment here.
    if(m_frameIndex % 20 == 0 && !m_candidates.empty()) try_schedule_relocalisation();
  }

  ++m_frameIndex;

#if defined(WITH_OPENCV) && 0
  cv::waitKey(1);
#endif
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void CollaborativeComponent::add_relocalisation_candidates()
{
  const std::vector<std::string> sceneIDs = m_context->get_scene_ids();
  const int sceneCount = static_cast<int>(sceneIDs.size());

  // Determine which scenes have started.
  std::deque<bool> scenesStarted;
  for(size_t i = 0; i < sceneCount; ++i)
  {
    scenesStarted.push_back(m_context->get_slam_state(sceneIDs[i])->get_view().get() != NULL);
  }

#if SAVE_REAL_IMAGES
  std::vector<std::pair<ITMFloatImage_Ptr,ITMUChar4Image_Ptr> > rgbdImages;
  for(size_t j = 0; j < sceneCount; ++j)
  {
    if(!scenesStarted[j]) continue;

    ITMFloatImage_Ptr depthJ;
    ITMUChar4Image_Ptr rgbJ;

    if(m_frameIndex >= 50 && m_frameIndex % 10 == 0 && ((m_frameIndex - 50) / 10) % sceneCount == j)
    {
      const View_CPtr viewJ = m_context->get_slam_state(sceneIDs[j])->get_view();
      viewJ->depth->UpdateHostFromDevice();
      viewJ->rgb->UpdateHostFromDevice();

      depthJ.reset(new ITMFloatImage(viewJ->depth->noDims, true, false));
      rgbJ.reset(new ITMUChar4Image(viewJ->rgb->noDims, true, false));

      depthJ->SetFrom(viewJ->depth, ITMFloatImage::CPU_TO_CPU);
      rgbJ->SetFrom(viewJ->rgb, ITMUChar4Image::CPU_TO_CPU);
    }

    rgbdImages.push_back(std::make_pair(depthJ, rgbJ));
  }
#endif

  for(size_t i = 0; i < sceneCount; ++i)
  {
    if(!scenesStarted[i]) continue;

    for(size_t j = 0; j < sceneCount; ++j)
    {
      if(j == i) continue;
      if(!scenesStarted[j]) continue;
      if(!(m_frameIndex >= 50 && m_frameIndex % 10 == 0 && ((m_frameIndex - 50) / 10) % sceneCount == j)) continue;

      const std::string sceneI = sceneIDs[i];
      const std::string sceneJ = sceneIDs[j];

      boost::optional<CollaborativeContext::SE3PoseCluster> largestCluster = m_context->try_get_largest_cluster(sceneI, sceneJ);
      const bool redundant = largestCluster && largestCluster->size() >= m_maxRelocalisationsNeeded;

      if(!redundant)
      {
        const SLAMState_CPtr slamStateJ = m_context->get_slam_state(sceneJ);
        SubmapRelocalisation_Ptr candidate(
          new SubmapRelocalisation(
            sceneI, sceneJ, m_frameIndex, slamStateJ->get_intrinsics().projectionParamsSimple.all, slamStateJ->get_pose()
#if SAVE_REAL_IMAGES
            , rgbdImages[j].first, rgbdImages[j].second
#endif
          )
        );
        (redundant ? m_redundantCandidates : m_candidates).push_back(std::make_pair(candidate, 0.0f));
      }
    }
  }
}

void CollaborativeComponent::run_relocalisation()
{
  const float failurePenaltyIncrease = 1.0f;
  const float failurePenaltyMax = 5.0f;

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

    std::cout << "Attempting to relocalise " << m_bestCandidate->m_sceneJ << " against " << m_bestCandidate->m_sceneI << "...";
    Relocaliser_CPtr relocaliserI = m_context->get_relocaliser(m_bestCandidate->m_sceneI);

    const SLAMState_CPtr slamStateJ = m_context->get_slam_state(m_bestCandidate->m_sceneJ);
    ITMFloatImage_Ptr depth(new ITMFloatImage(slamStateJ->get_depth_image_size(), true, true));
    ITMUChar4Image_Ptr rgb(new ITMUChar4Image(slamStateJ->get_rgb_image_size(), true, true));
#if USE_REAL_IMAGES
    depth->SetFrom(m_bestCandidate->m_depthJ.get(), ITMFloatImage::CPU_TO_CPU);
    rgb->SetFrom(m_bestCandidate->m_rgbJ.get(), ITMUChar4Image::CPU_TO_CPU);
#else
    VoxelRenderState_Ptr renderState;
    m_context->get_visualisation_generator()->generate_depth_from_voxels(
      depth, slamStateJ->get_voxel_scene(), m_bestCandidate->m_localPoseJ, slamStateJ->get_view(), renderState, DepthVisualiser::DT_ORTHOGRAPHIC
    );
    m_context->get_visualisation_generator()->generate_voxel_visualisation(
      rgb, slamStateJ->get_voxel_scene(), m_bestCandidate->m_localPoseJ, slamStateJ->get_view(), renderState, VisualisationGenerator::VT_SCENE_COLOUR
    );
#endif
    depth->UpdateDeviceFromHost();
    rgb->UpdateDeviceFromHost();

#if defined(WITH_OPENCV) && 0
    ITMUChar4Image_Ptr temp(new ITMUChar4Image(depth->noDims, true, false));
    ITMLib::IITMVisualisationEngine::DepthToUchar4(temp.get(), depth.get());

    cv::Mat3b cvRGB = OpenCVUtil::make_rgb_image(rgb->GetData(MEMORYDEVICE_CPU), rgb->noDims.x, rgb->noDims.y);
    cv::imshow("Relocalisation RGB", cvRGB);
    cv::Mat3b cvDepth = OpenCVUtil::make_rgb_image(temp->GetData(MEMORYDEVICE_CPU), temp->noDims.x, temp->noDims.y);
    cv::imshow("Relocalisation Depth", cvDepth);
    cv::waitKey(1);
#endif

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

    m_triedLocalPoses[std::make_pair(m_bestCandidate->m_sceneI, m_bestCandidate->m_sceneJ)].push_back(m_bestCandidate->m_localPoseJ);

    // Note: We make a copy of the best candidate before resetting it so that the deallocation of memory can happen after the lock has been released.
    SubmapRelocalisation_Ptr bestCandidateCopy;
    {
      boost::unique_lock<boost::mutex> lock(m_mutex);
      bestCandidateCopy = m_bestCandidate;
      m_bestCandidate.reset();
    }
  }
}

void CollaborativeComponent::try_schedule_relocalisation()
{
  for(std::list<Candidate>::iterator it = m_candidates.begin(), iend = m_candidates.end(); it != iend; ++it)
  {
    SubmapRelocalisation_Ptr candidate = it->first;

    boost::optional<CollaborativeContext::SE3PoseCluster> largestCluster = m_context->try_get_largest_cluster(candidate->m_sceneI, candidate->m_sceneJ);
    size_t largestClusterSize = largestCluster ? largestCluster->size() : 0;
    float sizeTerm = m_maxRelocalisationsNeeded - static_cast<float>(largestClusterSize);

    float primaryTerm = candidate->m_sceneI == "World" || candidate->m_sceneJ == "World" ? 5.0f : 0.0f;

    float homogeneityPenalty = 0.0f;
    const std::vector<ORUtils::SE3Pose>& triedLocalPoses = m_triedLocalPoses[std::make_pair(candidate->m_sceneI, candidate->m_sceneJ)];
    for(size_t j = 0, size = triedLocalPoses.size(); j < size; ++j)
    {
      if(GeometryUtil::poses_are_similar(candidate->m_localPoseJ, triedLocalPoses[j]))
      {
        homogeneityPenalty = 5.0f;
        break;
      }
    }

    float score = sizeTerm + primaryTerm - m_failurePenalties[std::make_pair(candidate->m_sceneI, candidate->m_sceneJ)] - homogeneityPenalty;
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

  bool canRelocalise = false;

  {
    boost::unique_lock<boost::mutex> lock(m_mutex);
    if(!m_bestCandidate)
    {
      // Try to relocalise the best candidate.
      m_bestCandidate = m_candidates.back().first;
      m_candidates.pop_back();
      canRelocalise = true;
    }
  }

  if(canRelocalise) m_readyToRelocalise.notify_one();
}

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
