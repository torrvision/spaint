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

#define DEBUGGING 1

namespace spaint {

//#################### CONSTRUCTORS ####################

CollaborativeComponent::CollaborativeComponent(const CollaborativeContext_Ptr& context, CollaborationMode mode)
: m_context(context),
  m_frameIndex(0),
  m_mode(mode),
  m_rng(12345),
  m_stopRelocalisationThread(false)
{
  m_relocalisationThread = boost::thread(boost::bind(&CollaborativeComponent::run_relocalisation, this));
  m_context->get_pose_graph_optimiser()->start();
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
  bool fusionMayStillRun = update_trajectories();
  if(!fusionMayStillRun) m_mode = CM_BATCH;

  if(m_frameIndex > 0 && (!fusionMayStillRun || (m_mode == CM_LIVE && m_frameIndex % 20 == 0)))
  {
    try_schedule_relocalisation();
  }

  ++m_frameIndex;

#if defined(WITH_OPENCV) && 0
  cv::waitKey(1);
#endif
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

size_t CollaborativeComponent::count_orb_keypoints(const ITMUChar4Image *image, size_t defaultKeypointCount) const
{
#ifdef WITH_OPENCV
  cv::Mat3b cvImage = OpenCVUtil::make_rgb_image(image->GetData(MEMORYDEVICE_CPU), image->noDims.x, image->noDims.y);
  return OpenCVUtil::count_orb_keypoints(cvImage);
#else
  return defaultKeypointCount;
#endif
}

std::list<CollaborativeComponent::Candidate> CollaborativeComponent::generate_random_candidates(size_t desiredCandidateCount) const
{
  std::list<Candidate> candidates;

  const int sceneCount = static_cast<int>(m_trajectories.size());
  if(sceneCount < 2) return std::list<Candidate>();

  for(size_t i = 0; i < desiredCandidateCount; ++i)
  {
    // Randomly select the trajectories of two different scenes.
    int ki = m_rng.generate_int_from_uniform(0, sceneCount - 1);
    int kj = m_rng.generate_int_from_uniform(0, sceneCount - 2);
    if(kj >= ki) ++kj;

    std::map<std::string,std::deque<ORUtils::SE3Pose> >::const_iterator it = m_trajectories.begin();
    std::map<std::string,std::deque<ORUtils::SE3Pose> >::const_iterator jt = m_trajectories.begin();
    std::advance(it, ki);
    std::advance(jt, kj);

    const std::string sceneI = it->first;
    const std::string sceneJ = jt->first;

    // Randomly pick a frame from scene j.
    SLAMState_CPtr slamStateJ = m_context->get_slam_state(sceneJ);
    const int frameCountJ = static_cast<int>(jt->second.size());
    const int frameIndexJ = m_mode == CM_BATCH || slamStateJ->get_input_status() != SLAMState::IS_ACTIVE
      ? m_rng.generate_int_from_uniform(0, frameCountJ - 1)
      : frameCountJ - 1;

    // Add a candidate to relocalise the selected frame of scene j against scene i.
    const Vector4f& depthIntrinsicsJ = slamStateJ->get_intrinsics().projectionParamsSimple.all;
    const ORUtils::SE3Pose& localPoseJ = jt->second[frameIndexJ];

    SubmapRelocalisation_Ptr candidate(new SubmapRelocalisation(sceneI, sceneJ, frameIndexJ, depthIntrinsicsJ, localPoseJ));
    candidates.push_back(std::make_pair(candidate, 0.0f));
  }

  return candidates;
}

void CollaborativeComponent::run_relocalisation()
{
  while(!m_stopRelocalisationThread)
  {
    // Wait for a relocalisation to be scheduled.
    {
      boost::unique_lock<boost::mutex> lock(m_mutex);
      while(!m_bestCandidate)
      {
        m_readyToRelocalise.wait(lock);

        // TODO: Comment here.
        if(m_stopRelocalisationThread) return;
      }
    }

    std::cout << "Attempting to relocalise frame " << m_bestCandidate->m_frameIndexJ << " of " << m_bestCandidate->m_sceneJ << " against " << m_bestCandidate->m_sceneI << "...";

    // Render synthetic images of the source scene from the relevant pose and copy them across to the GPU for use by the relocaliser.
    const SLAMState_CPtr slamStateJ = m_context->get_slam_state(m_bestCandidate->m_sceneJ);
    ITMFloatImage_Ptr depth(new ITMFloatImage(slamStateJ->get_depth_image_size(), true, true));
    ITMUChar4Image_Ptr rgb(new ITMUChar4Image(slamStateJ->get_rgb_image_size(), true, true));

    VoxelRenderState_Ptr renderStateD;
    m_context->get_visualisation_generator()->generate_depth_from_voxels(
      depth, slamStateJ->get_voxel_scene(), m_bestCandidate->m_localPoseJ, slamStateJ->get_view(), renderStateD, DepthVisualiser::DT_ORTHOGRAPHIC
    );

    VoxelRenderState_Ptr renderStateRGB;
    const bool useColourIntrinsics = true;
    m_context->get_visualisation_generator()->generate_voxel_visualisation(
      rgb, slamStateJ->get_voxel_scene(), m_bestCandidate->m_localPoseJ, slamStateJ->get_view(),
      renderStateRGB, VisualisationGenerator::VT_SCENE_COLOUR, boost::none, useColourIntrinsics
    );

    depth->UpdateDeviceFromHost();
    rgb->UpdateDeviceFromHost();

#ifdef WITH_OPENCV
    // Make OpenCV copies of the synthetic images we're trying to relocalise (these may be needed later).
    cv::Mat3b cvSourceRGB = OpenCVUtil::make_rgb_image(rgb->GetData(MEMORYDEVICE_CPU), rgb->noDims.x, rgb->noDims.y);
    cv::Mat1b cvSourceDepth = OpenCVUtil::make_greyscale_image(depth->GetData(MEMORYDEVICE_CPU), depth->noDims.x, depth->noDims.y, OpenCVUtil::ROW_MAJOR, 100.0f);

  #if DEBUGGING
    // If we're debugging, show the synthetic images of the source scene to the user.
    cv::imshow("Source Depth", cvSourceDepth);
    cv::imshow("Source RGB", cvSourceRGB);
  #endif
#endif

    // Attempt to relocalise the synthetic images using the relocaliser for the target scene.
    Relocaliser_CPtr relocaliserI = m_context->get_relocaliser(m_bestCandidate->m_sceneI);
    boost::optional<Relocaliser::Result> result = relocaliserI->relocalise(rgb.get(), depth.get(), m_bestCandidate->m_depthIntrinsicsJ);

    // If relocalisation succeeded, verify the result by thresholding the difference between the
    // source depth image and a rendered depth image of the target scene at the relevant pose.
    bool verified = false;
    cv::Scalar meanDepthDiff;
    if(result && result->quality == Relocaliser::RELOCALISATION_GOOD)
    {
#ifdef WITH_OPENCV
      // Render synthetic images of the target scene from the relevant pose.
      const SLAMState_CPtr slamStateI = m_context->get_slam_state(m_bestCandidate->m_sceneI);
      renderStateD.reset();
      renderStateRGB.reset();

      m_context->get_visualisation_generator()->generate_depth_from_voxels(
        depth, slamStateI->get_voxel_scene(), result->pose.GetM(), slamStateI->get_view(), renderStateD, DepthVisualiser::DT_ORTHOGRAPHIC
      );

      m_context->get_visualisation_generator()->generate_voxel_visualisation(
        rgb, slamStateI->get_voxel_scene(), result->pose.GetM(), slamStateI->get_view(),
        renderStateRGB, VisualisationGenerator::VT_SCENE_COLOUR, boost::none, useColourIntrinsics
      );

      // Make OpenCV copies of the synthetic images of the target scene.
      cv::Mat3b cvTargetRGB = OpenCVUtil::make_rgb_image(rgb->GetData(MEMORYDEVICE_CPU), rgb->noDims.x, rgb->noDims.y);
      cv::Mat1b cvTargetDepth = OpenCVUtil::make_greyscale_image(depth->GetData(MEMORYDEVICE_CPU), depth->noDims.x, depth->noDims.y, OpenCVUtil::ROW_MAJOR, 100.0f);

    #if DEBUGGING
      // If we're debugging, show the synthetic images of the target scene to the user.
      cv::imshow("Target RGB", cvTargetRGB);
      cv::imshow("Target Depth", cvTargetDepth);
    #endif

      // Compute a binary mask showing which pixels are valid in both the source and target depth images.
      cv::Mat cvSourceMask;
      cv::inRange(cvSourceDepth, cv::Scalar(0,0,0), cv::Scalar(0,0,0), cvSourceMask);
      cv::bitwise_not(cvSourceMask, cvSourceMask);

      cv::Mat cvTargetMask;
      cv::inRange(cvTargetDepth, cv::Scalar(0,0,0), cv::Scalar(0,0,0), cvTargetMask);
      cv::bitwise_not(cvTargetMask, cvTargetMask);

      cv::Mat cvCombinedMask;
      cv::bitwise_and(cvSourceMask, cvTargetMask, cvCombinedMask);

      // Compute the difference between the source and target depth images, and mask it using the combined mask.
      cv::Mat cvDepthDiff, cvMaskedDepthDiff;
      cv::absdiff(cvSourceDepth, cvTargetDepth, cvDepthDiff);
      cvDepthDiff.copyTo(cvMaskedDepthDiff, cvCombinedMask);
    #if DEBUGGING
      cv::imshow("Masked Depth Difference", cvMaskedDepthDiff);
    #endif

      // Determine the average depth difference for valid pixels in the source and target depth images.
      meanDepthDiff = cv::mean(cvMaskedDepthDiff);
    #if DEBUGGING
      std::cout << "\nMean Depth Difference: " << meanDepthDiff << std::endl;
    #endif

      // Compute the fraction of the target depth image that is valid.
      const float targetValidFraction = static_cast<float>(cv::countNonZero(cvTargetMask == 255)) / (cvTargetMask.size().width * cvTargetMask.size().height);
    #if DEBUGGING
      std::cout << "Valid Target Pixels: " << cv::countNonZero(cvTargetMask == 255) << std::endl;
    #endif

      // Decide whether or not to verify the relocalisation, based on the average depth difference and the fraction of the target depth image that is valid.
      verified = meanDepthDiff(0) < 5.0f && targetValidFraction >= 0.5f;
#else
      // If we didn't build with OpenCV, we can't do any verification, so just mark the relocalisation as verified and hope for the best.
      verified = true;
#endif
    }

    // If relocalisation succeeded and we successfully verified the result, add a sample of the
    // relative transform between the source and target scenes to the pose graph optimiser.
    if(verified)
    {
      // cjTwi^-1 * cjTwj = wiTcj * cjTwj = wiTwj
      m_bestCandidate->m_relativePose = ORUtils::SE3Pose(result->pose.GetInvM() * m_bestCandidate->m_localPoseJ.GetM());
      m_context->get_pose_graph_optimiser()->add_relative_transform_sample(m_bestCandidate->m_sceneI, m_bestCandidate->m_sceneJ, *m_bestCandidate->m_relativePose, m_mode);
      std::cout << "succeeded!" << std::endl;

#ifdef WITH_OPENCV
      cv::waitKey(1);
#endif
    }
    else
    {
      std::cout << "failed :(" << std::endl;

#ifdef WITH_OPENCV
      cv::waitKey(1);
#endif
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

void CollaborativeComponent::score_candidates(std::list<Candidate>& candidates) const
{
  for(std::list<Candidate>::iterator it = candidates.begin(), iend = candidates.end(); it != iend; ++it)
  {
    SubmapRelocalisation_Ptr candidate = it->first;

    // Boost candidates that may attach new confident nodes to the graph.
    PoseGraphOptimiser_CPtr poseGraphOptimiser = m_context->get_pose_graph_optimiser();
    boost::optional<ORUtils::SE3Pose> globalPoseI = poseGraphOptimiser->try_get_estimated_global_pose(candidate->m_sceneI);
    boost::optional<ORUtils::SE3Pose> globalPoseJ = poseGraphOptimiser->try_get_estimated_global_pose(candidate->m_sceneJ);
    float newNodeBoost = (globalPoseI && !globalPoseJ) || (globalPoseJ && !globalPoseI) ? 1.0f : 0.0f;

    // Penalise candidates that will only add to an existing confident edge.
    boost::optional<PoseGraphOptimiser::SE3PoseCluster> largestCluster = poseGraphOptimiser->try_get_largest_cluster(candidate->m_sceneI, candidate->m_sceneJ);
    int largestClusterSize = largestCluster ? static_cast<int>(largestCluster->size()) : 0;
    float confidencePenalty = 1.0f * std::max(largestClusterSize - PoseGraphOptimiser::confidence_threshold(), 0);

    // Penalise candidates that are too close to ones we've tried before.
    float homogeneityPenalty = 0.0f;
    std::map<std::pair<std::string,std::string>,std::deque<int> >::const_iterator jt = m_triedFrameIndices.find(std::make_pair(candidate->m_sceneI, candidate->m_sceneJ));
    if(jt != m_triedFrameIndices.end())
    {
      const std::deque<int>& triedFrameIndices = jt->second;
      for(size_t j = 0, size = triedFrameIndices.size(); j < size; ++j)
      {
        const ORUtils::SE3Pose& triedPose = m_trajectories.find(candidate->m_sceneJ)->second[triedFrameIndices[j]];
        if(GeometryUtil::poses_are_similar(candidate->m_localPoseJ, triedPose, 5 * M_PI / 180))
        {
          homogeneityPenalty = 5.0f;
          break;
        }
      }
    }

    float score = newNodeBoost - confidencePenalty - homogeneityPenalty;
    it->second = score;
  }
}

void CollaborativeComponent::try_schedule_relocalisation()
{
  // Randomly generate a list of candidate relocalisations.
  const size_t desiredCandidateCount = 10;
  std::list<Candidate> candidates = generate_random_candidates(desiredCandidateCount);
  if(candidates.empty()) return;

  // Score all of the candidates.
  score_candidates(candidates);

  // Sort the candidates in ascending order of score (this isn't strictly necessary, but makes debugging easier).
  candidates.sort(bind(&Candidate::second, _1) < bind(&Candidate::second, _2));

#if 0
  // Print out all of the candidates for debugging purposes.
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
      // Schedule the best candidate for relocalisation.
      m_bestCandidate = candidates.back().first;
      candidates.pop_back();
      canRelocalise = true;

      // Record the index of the frame we're trying in case we want to avoid frames with similar poses later.
      std::deque<int>& triedFrameIndices = m_triedFrameIndices[std::make_pair(m_bestCandidate->m_sceneI, m_bestCandidate->m_sceneJ)];
      triedFrameIndices.push_back(m_bestCandidate->m_frameIndexJ);
    }
  }

  if(canRelocalise) m_readyToRelocalise.notify_one();
}

bool CollaborativeComponent::update_trajectories()
{
  bool fusionMayStillRun = false;

  const std::vector<std::string> sceneIDs = m_context->get_scene_ids();
  for(size_t i = 0, sceneCount = sceneIDs.size(); i < sceneCount; ++i)
  {
    SLAMState_CPtr slamState = m_context->get_slam_state(sceneIDs[i]);
    if(!slamState || !slamState->get_view()) continue;

    SLAMState::InputStatus inputStatus = slamState->get_input_status();
    TrackingState_CPtr trackingState = slamState->get_tracking_state();
    const ITMUChar4Image *rgbImage = slamState->get_view()->rgb;
    rgbImage->UpdateHostFromDevice();
    if(inputStatus == SLAMState::IS_ACTIVE && trackingState->trackerResult == ITMTrackingState::TRACKING_GOOD && count_orb_keypoints(rgbImage) >= 400)
    {
      m_trajectories[sceneIDs[i]].push_back(*trackingState->pose_d);
    }

    if(inputStatus != SLAMState::IS_TERMINATED) fusionMayStillRun = true;
  }

  return fusionMayStillRun;
}

}
