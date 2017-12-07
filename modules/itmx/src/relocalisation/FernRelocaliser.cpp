/**
 * itmx: FernRelocaliser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "relocalisation/FernRelocaliser.h"

#include <boost/filesystem.hpp>
namespace bf = boost::filesystem;

namespace itmx {

//#################### CONSTRUCTORS ####################

FernRelocaliser::FernRelocaliser(const Vector2i& depthImageSize, float viewFrustumMin, float viewFrustumMax,
                                 float harvestingThreshold, int numFerns, int decisionsPerFern,
                                 KeyframeAddPolicy keyframeAddPolicy)
: m_decisionsPerFern(decisionsPerFern),
  m_depthImageSize(depthImageSize),
  m_harvestingThreshold(harvestingThreshold),
  m_keyframeAddPolicy(keyframeAddPolicy),
  m_numFerns(numFerns),
  m_rangeParameters(viewFrustumMin, viewFrustumMax)
{
  reset();
}

//#################### PUBLIC VIRTUAL MEMBER FUNCTIONS ####################

void FernRelocaliser::load_from_disk(const std::string& inputFolder)
{
  // Load data from disk.
  // Note that we need to add a "/" to the end of the folder to force the loading to happen from INSIDE the folder.
  m_relocaliser->LoadFromDirectory(inputFolder + "/");
}

boost::optional<Relocaliser::Result>
FernRelocaliser::relocalise(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage, const Vector4f &depthIntrinsics) const
{
  // Copy the current depth input across to the CPU for use by the relocaliser.
  depthImage->UpdateHostFromDevice();

  // Since we are relocalising, we don't want to add this as a keyframe.
  bool considerKeyframe = false;
  const int sceneId = 0;
  const int requestedNearestNeighbourCount = 1;
  int nearestNeighbour = -1;

  // Process the current depth image using the relocaliser. This attempts to find the nearest keyframe
  // (if any) that is currently in the database.
  m_relocaliser->ProcessFrame(depthImage, NULL, sceneId, requestedNearestNeighbourCount, &nearestNeighbour, NULL, considerKeyframe);

  // If a nearest keyframe was found by the relocaliser, return it.
  if(nearestNeighbour != -1)
  {
    // Set the number of frames for which the train function has to be called before the relocaliser can
    // consider adding a new keyframe (no need to check the policy here).
    m_keyframeDelay = 10;

    // Retrieve the pose and set the quality to GOOD.
    Result result;
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
    m_depthImageSize, m_rangeParameters, m_harvestingThreshold, m_numFerns, m_decisionsPerFern
                        ));
}

void FernRelocaliser::save_to_disk(const std::string& outputFolder) const
{
  // First, make sure the output folder exists.
  bf::create_directories(outputFolder);

  // Then save the data.
  // Note that we have to add the "/" to the folder to force the writing to happen INSIDE that folder.
  m_relocaliser->SaveToDirectory(outputFolder + "/");
}

void FernRelocaliser::train(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage,
                            const Vector4f& depthIntrinsics, const ORUtils::SE3Pose& cameraPose)
{
  // If this function is being called, the assumption is that tracking succeeded and that we could
  // in principle add this frame as a keyframe. We will actually add it in practice if either
  // (a) our policy is to always add keyframes, or (b) our policy is to add keyframes once a
  // suitable delay has elapsed since the most recent relocalisation, and that delay actually
  // has elapsed. Conversely, if the delay has not yet elapsed, we decrease the delay counter
  // and early out.
  if(m_keyframeAddPolicy == DELAY_AFTER_RELOCALISATION && m_keyframeDelay > 0)
  {
    --m_keyframeDelay;
    return;
  }

  // Copy the current depth input across to the CPU for use by the relocaliser.
  depthImage->UpdateHostFromDevice();

  // Process the current depth image using the relocaliser. This attempts to find the nearest keyframe
  // (if any) that is currently in the database, and may add the current frame as a new keyframe if the
  // current frame differs sufficiently from the existing keyframes.
  const bool considerKeyframe = true;
  const int sceneId = 0;
  const int requestedNearestNeighbourCount = 1;
  int nearestNeighbour = -1;

  m_relocaliser->ProcessFrame(depthImage, &cameraPose, sceneId, requestedNearestNeighbourCount, &nearestNeighbour, NULL, considerKeyframe);
}

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

float FernRelocaliser::get_default_harvesting_threshold()
{
  return 0.2f; // from InfiniTAM
}

FernRelocaliser::KeyframeAddPolicy FernRelocaliser::get_default_keyframe_add_policy()
{
  return DELAY_AFTER_RELOCALISATION; // standard behaviour
}

int FernRelocaliser::get_default_num_decisions_per_fern()
{
  return 4; // from InfiniTAM
}

int FernRelocaliser::get_default_num_ferns()
{
  return 500; // from InfiniTAM
}

}
