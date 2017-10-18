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

#include <tvgutil/numbers/RandomNumberGenerator.h>
using namespace tvgutil;

#define USE_REAL_IMAGES 0
#define SAVE_REAL_IMAGES USE_REAL_IMAGES

namespace spaint {

//#################### CONSTRUCTORS ####################

CollaborativeComponent::CollaborativeComponent(const CollaborativeContext_Ptr& context)
: m_context(context),
  m_frameIndex(0),
  m_nextScenePairIndex(0),
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
    add_relocalisation_candidates();

    // TODO: Comment here.
    if(m_frameIndex % 20 == 0) try_schedule_relocalisation();
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
      //if(!(m_frameIndex >= 50 && m_frameIndex % 10 == 0 && ((m_frameIndex - 50) / 10) % sceneCount == j)) continue;

      const std::string sceneI = sceneIDs[i];
      const std::string sceneJ = sceneIDs[j];

      const SLAMState_CPtr slamStateJ = m_context->get_slam_state(sceneJ);
      SubmapRelocalisation_Ptr candidate(
        new SubmapRelocalisation(
          sceneI, sceneJ, m_frameIndex, slamStateJ->get_intrinsics().projectionParamsSimple.all, slamStateJ->get_pose()
#if SAVE_REAL_IMAGES
          , rgbdImages[j].first, rgbdImages[j].second
#endif
        )
      );

      m_scenePairInfos[std::make_pair(sceneI, sceneJ)].m_candidates.push_back(std::make_pair(candidate, 0.0f));
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

    std::cout << "Attempting to relocalise frame " << m_bestCandidate->m_frameIndex << " of " << m_bestCandidate->m_sceneJ << " against " << m_bestCandidate->m_sceneI << "...";
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
      m_context->get_pose_graph_optimiser()->add_relative_transform_sample(m_bestCandidate->m_sceneI, m_bestCandidate->m_sceneJ, *m_bestCandidate->m_relativePose);
      m_scenePairInfos[std::make_pair(m_bestCandidate->m_sceneI, m_bestCandidate->m_sceneJ)].m_triedLocalPoses.push_back(m_bestCandidate->m_localPoseJ);
      std::cout << "succeeded!\n";
    }
    else
    {
      std::cout << "failed :(\n";
    }

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
  if(m_scenePairInfos.empty()) return;

  std::map<std::pair<std::string,std::string>,ScenePairInfo>::iterator kt = m_scenePairInfos.end();
  size_t numTries = 0;
  while(kt == m_scenePairInfos.end() && numTries < m_scenePairInfos.size())
  {
    kt = m_scenePairInfos.begin();
    std::advance(kt, m_nextScenePairIndex);
    m_nextScenePairIndex = (m_nextScenePairIndex + 1) % m_scenePairInfos.size();
    ++numTries;

    boost::optional<PoseGraphOptimiser::SE3PoseCluster> largestCluster = m_context->get_pose_graph_optimiser()->try_get_largest_cluster(kt->first.first, kt->first.second);
    if(kt->second.m_candidates.empty()/* || (largestCluster && largestCluster->size() >= PoseGraphOptimiser::confidence_threshold())/* || (kt->first.first != "World" && kt->first.second != "World")*/)
    {
      kt = m_scenePairInfos.end();
    }
  }

  if(kt == m_scenePairInfos.end()) return;

  std::list<Candidate>& candidates = kt->second.m_candidates;
  if(candidates.empty()) return;

  static RandomNumberGenerator rng(12345);

  for(std::list<Candidate>::iterator it = candidates.begin(), iend = candidates.end(); it != iend; ++it)
  {
    SubmapRelocalisation_Ptr candidate = it->first;

    float homogeneityPenalty = 0.0f;
    const std::vector<ORUtils::SE3Pose>& triedLocalPoses = m_scenePairInfos[std::make_pair(candidate->m_sceneI, candidate->m_sceneJ)].m_triedLocalPoses;
    for(size_t j = 0, size = triedLocalPoses.size(); j < size; ++j)
    {
      if(GeometryUtil::poses_are_similar(candidate->m_localPoseJ, triedLocalPoses[j]))
      {
        homogeneityPenalty = 5.0f;
        break;
      }
    }

    float score = -homogeneityPenalty + rng.generate_real_from_uniform<float>(-1.0f, 1.0f);
    it->second = score;
  }

  candidates.sort(bind(&Candidate::second, _1) < bind(&Candidate::second, _2));

#if 0
  std::cout << "BEGIN CANDIDATES " << m_frameIndex << '\n';
  for(std::list<Candidate>::const_iterator it = candidates.begin(), iend = candidates.end(); it != iend; ++it)
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
      m_bestCandidate = candidates.back().first;
      candidates.pop_back();
      canRelocalise = true;
    }
  }

  if(canRelocalise) m_readyToRelocalise.notify_one();
}

}
