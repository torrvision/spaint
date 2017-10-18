/**
 * spaint: CollaborativeComponent.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "pipelinecomponents/CollaborativeComponent.h"
using namespace ITMLib;
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
  update_trajectories();

  if(m_frameIndex > 0 && m_frameIndex % 20 == 0)
  {
    try_schedule_relocalisation();
  }

  ++m_frameIndex;

#if defined(WITH_OPENCV) && 0
  cv::waitKey(1);
#endif
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

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
  // Randomly generate a list of candidate relocalisations.
  std::list<Candidate> candidates;
  const size_t desiredCandidateCount = 1;

  const int sceneCount = static_cast<int>(m_trajectories.size());
  if(sceneCount < 2) return;

  static RandomNumberGenerator rng(12345);
  for(size_t i = 0; i < desiredCandidateCount; ++i)
  {
    int ki = rng.generate_int_from_uniform(0, sceneCount - 1);
    int kj = rng.generate_int_from_uniform(0, sceneCount - 2);
    if(kj >= ki) ++kj;

    std::map<std::string,std::deque<ORUtils::SE3Pose> >::const_iterator it = m_trajectories.begin();
    std::map<std::string,std::deque<ORUtils::SE3Pose> >::const_iterator jt = m_trajectories.begin();

    std::advance(it, ki);
    std::advance(jt, kj);

    const std::string sceneI = it->first, sceneJ = jt->first;

    const int frameCount = static_cast<int>(jt->second.size());
    const int frameIndex = rng.generate_int_from_uniform(0, frameCount - 1);

    SLAMState_CPtr slamStateJ = m_context->get_slam_state(sceneJ);
    const Vector4f& depthIntrinsicsJ = slamStateJ->get_intrinsics().projectionParamsSimple.all;

    const ORUtils::SE3Pose& localPoseJ = jt->second[frameIndex];

    SubmapRelocalisation_Ptr candidate(new SubmapRelocalisation(sceneI, sceneJ, frameIndex, depthIntrinsicsJ, localPoseJ));
    candidates.push_back(std::make_pair(candidate, 0.0f));
  }

  // Score all of the candidates.
  for(std::list<Candidate>::iterator it = candidates.begin(), iend = candidates.end(); it != iend; ++it)
  {
    SubmapRelocalisation_Ptr candidate = it->first;
    float score = rng.generate_real_from_uniform<float>(-1.0f, 1.0f);
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

void CollaborativeComponent::update_trajectories()
{
  const std::vector<std::string> sceneIDs = m_context->get_scene_ids();
  for(size_t i = 0, sceneCount = sceneIDs.size(); i < sceneCount; ++i)
  {
    SLAMState_CPtr slamState = m_context->get_slam_state(sceneIDs[i]);
    TrackingState_CPtr trackingState = slamState->get_tracking_state();
    if(slamState->get_frame_processed() && trackingState->trackerResult == ITMTrackingState::TRACKING_GOOD)
    {
      m_trajectories[sceneIDs[i]].push_back(*trackingState->pose_d);
    }
  }
}

}
