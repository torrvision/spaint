/**
 * itmx: FernRelocaliser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "relocalisation/FernRelocaliser.h"

namespace itmx {

//#################### CONSTRUCTORS ####################

FernRelocaliser::FernRelocaliser(Vector2i depthImageSize,
                                 float viewFrustumMin,
                                 float viewFrustumMax,
                                 float harvestingThreshold,
                                 int numFerns,
                                 int decisionsPerFern,
                                 KeyframeAddPolicy keyframeAddPolicy)
{
  m_decisionsPerFern = decisionsPerFern;
  m_depthImageSize = depthImageSize;
  m_harvestingThreshold = harvestingThreshold;
  m_keyframeAddPolicy = keyframeAddPolicy;
  m_numFerns = numFerns;
  m_rangeParameters = Vector2f(viewFrustumMin, viewFrustumMax);

  reset();
}

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

float FernRelocaliser::get_default_harvesting_threshold()
{
  return 0.2f; // From InfiniTAM.
}

FernRelocaliser::KeyframeAddPolicy FernRelocaliser::get_default_keyframe_add_policy()
{
  return DELAY_AFTER_RELOCALISATION; // Standard behaviour.
}

int FernRelocaliser::get_default_num_decisions_per_fern()
{
  return 4; // From InfiniTAM.
}

int FernRelocaliser::get_default_num_ferns()
{
  return 500; // From InfiniTAM.
}

//#################### PUBLIC VIRTUAL MEMBER FUNCTIONS ####################

void FernRelocaliser::integrate_rgbd_pose_pair(const ITMUChar4Image * /* dummy */,
                                               const ITMFloatImage *depthImage,
                                               const Vector4f & /* dummy */,
                                               const ORUtils::SE3Pose &cameraPose)
{
  // If this function is being called the assumption is that tracking succeeded, so we always consider this frame to be
  // a keyframe unless we just relocalised and the policy specifies that we have to wait, in that case early out.
  if (m_keyframeDelay > 0 && m_keyframeAddPolicy == DELAY_AFTER_RELOCALISATION)
  {
    --m_keyframeDelay;
    return;
  }

  // Copy the current depth input across to the CPU for use by the relocaliser.
  depthImage->UpdateHostFromDevice();

  // Process the current depth image using the relocaliser. This attempts to find the nearest keyframe (if any)
  // that is currently in the database, and may add the current frame as a new keyframe if the current frame differs
  // sufficiently from the existing keyframes.

  const bool considerKeyframe = true;
  const int sceneId = 0;
  const int requestedNnCount = 1;
  int nearestNeighbour = -1;
  m_relocaliser->ProcessFrame(
      depthImage, &cameraPose, sceneId, requestedNnCount, &nearestNeighbour, NULL, considerKeyframe);
}

boost::optional<Relocaliser::RelocalisationResult> FernRelocaliser::relocalise(const ITMUChar4Image *colourImage,
                                                                               const ITMFloatImage *depthImage,
                                                                               const Vector4f &depthIntrinsics) const
{
  // Copy the current depth input across to the CPU for use by the relocaliser.
  depthImage->UpdateHostFromDevice();

  // Since we are relocalising, we don't want to add this as a keyframe.
  bool considerKeyframe = false;
  const int sceneId = 0;
  const int requestedNnCount = 1;
  int nearestNeighbour = -1;

  // Process the current depth image using the relocaliser. This attempts to find the nearest keyframe (if any)
  // that is currently in the database.
  m_relocaliser->ProcessFrame(depthImage, NULL, sceneId, requestedNnCount, &nearestNeighbour, NULL, considerKeyframe);

  // If a nearest keyframe was found by the relocaliser, return it.
  if (nearestNeighbour != -1)
  {
    // Set the number of frames for which the  integrate function has to be called before the relocaliser can consider
    // adding a new keyframe (no need to check the policy here).
    m_keyframeDelay = 10;

    // Retrieve the pose and set the quality to GOOD.
    RelocalisationResult result;
    result.pose = m_relocaliser->RetrievePose(nearestNeighbour).pose;
    result.quality = RELOCALISATION_GOOD;

    return result;
  }

  return boost::none;
}

void FernRelocaliser::reset()
{
  m_keyframeDelay = 0;
  m_relocaliser.reset(new WrappedRelocaliser(
      m_depthImageSize, m_rangeParameters, m_harvestingThreshold, m_numFerns, m_decisionsPerFern));
}

void FernRelocaliser::update()
{
  // Nothing to do for this relocaliser.
}

} // namespace itmx
