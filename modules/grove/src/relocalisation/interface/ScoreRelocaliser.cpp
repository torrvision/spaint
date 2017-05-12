/**
 * grove: ScoreRelocaliser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "relocalisation/interface/ScoreRelocaliser.h"

#include <itmx/base/MemoryBlockFactory.h>
using namespace itmx;

namespace grove {

//#################### CONSTRUCTORS ####################

ScoreRelocaliser::ScoreRelocaliser(const std::string &forestFilename)
{
  // In this constructor we are just setting the variables, instantiation of the sub-algorithms is left to the sub class
  // in order to instantiate the appropriate version.

  //
  // Forest
  //
  m_forestFilename = forestFilename;

  //
  // Reservoirs
  //
  m_maxReservoirsToUpdate = 256; // Update the modes associated to 256 reservoirs each integration/update call.
  // m_reservoirsCount is not set since that number depends on the forest that will be instantiated in the subclass.
  m_reservoirsCapacity = 1024;
  m_rngSeed = 42;

  //
  // Clustering
  //
  // Tentative values that seem to work.
  m_clustererSigma = 0.1f;
  m_clustererTau = 0.05f;
  m_maxClusterCount = ScorePrediction::MAX_CLUSTERS;
  m_minClusterSize = 20;

  MemoryBlockFactory &mbf = MemoryBlockFactory::instance();
  // Setup memory blocks/images (except m_predictionsBlock since its size depends on the forest)
  m_leafIndicesImage = mbf.make_image<LeafIndices>();
  m_predictionsImage = mbf.make_image<ScorePrediction>();
  m_rgbdPatchDescriptorImage = mbf.make_image<DescriptorType>();
  m_rgbdPatchKeypointsImage = mbf.make_image<ExampleType>();
}

//#################### DESTRUCTOR ####################

ScoreRelocaliser::~ScoreRelocaliser() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void ScoreRelocaliser::get_best_poses(std::vector<PoseCandidate> &poseCandidates) const
{
  // Just forward the vector to P-RANSAC.
  m_preemptiveRansac->get_best_poses(poseCandidates);
}

Keypoint3DColourImage_CPtr ScoreRelocaliser::get_keypoints_image() const { return m_rgbdPatchKeypointsImage; }

ScorePredictionsImage_CPtr ScoreRelocaliser::get_predictions_image() const { return m_predictionsImage; }

void ScoreRelocaliser::integrate_rgbd_pose_pair(const ITMUChar4Image *colourImage,
                                                const ITMFloatImage *depthImage,
                                                const Vector4f &depthIntrinsics,
                                                const ORUtils::SE3Pose &cameraPose)
{
  // First: select keypoints and compute descriptors.
  const Matrix4f invCameraPose = cameraPose.GetInvM();
  m_featureCalculator->compute_keypoints_and_features(colourImage,
                                                      depthImage,
                                                      invCameraPose,
                                                      depthIntrinsics,
                                                      m_rgbdPatchKeypointsImage.get(),
                                                      m_rgbdPatchDescriptorImage.get());

  // Second: find the leaves associated to the keypoints.
  m_scoreForest->find_leaves(m_rgbdPatchDescriptorImage, m_leafIndicesImage);

  // Third: add keypoints to the correct reservoirs.
  m_exampleReservoirs->add_examples(m_rgbdPatchKeypointsImage, m_leafIndicesImage);

  // Fourth: cluster some of the reservoirs.
  const uint32_t updateCount = compute_nb_reservoirs_to_update();
  m_exampleClusterer->find_modes(m_exampleReservoirs->get_reservoirs(),
                                 m_exampleReservoirs->get_reservoirs_size(),
                                 m_predictionsBlock,
                                 m_reservoirUpdateStartIdx,
                                 updateCount);

  // Fifth: save the current index to indicate that reservoirs up to such index have to be clustered to represent the
  // examples that have just been added.
  m_lastFeaturesAddedStartIdx = m_reservoirUpdateStartIdx;

  // Finally: update starting index for the next invocation of either this function or idle_update().
  update_reservoir_start_idx();
}

boost::optional<Relocaliser::RelocalisationResult> ScoreRelocaliser::relocalise(const ITMUChar4Image *colourImage,
                                                                                const ITMFloatImage *depthImage,
                                                                                const Vector4f &depthIntrinsics)
{
  // Try to estimate a pose only if we have enough valid depth values.
  if (m_lowLevelEngine->CountValidDepths(depthImage) > m_preemptiveRansac->get_min_nb_required_points())
  {
    // First: select keypoints and compute descriptors.
    m_featureCalculator->compute_keypoints_and_features(
        colourImage, depthImage, depthIntrinsics, m_rgbdPatchKeypointsImage.get(), m_rgbdPatchDescriptorImage.get());

    // Second: find all the leaves associated to the keypoints.
    m_scoreForest->find_leaves(m_rgbdPatchDescriptorImage, m_leafIndicesImage);

    // Third: merge the predictions associated to those leaves.
    get_predictions_for_leaves(m_leafIndicesImage, m_predictionsBlock, m_predictionsImage);

    // Finally: perform RANSAC.
    boost::optional<PoseCandidate> poseCandidate =
        m_preemptiveRansac->estimate_pose(m_rgbdPatchKeypointsImage, m_predictionsImage);

    // If we succeeded, grab the transformation matrix, fill the SE3Pose and return a GOOD relocalisation result.
    if (poseCandidate)
    {
      RelocalisationResult result;
      result.pose.SetInvM(poseCandidate->cameraPose); // TODO: rename the poseCandidate member
      result.quality = RELOCALISATION_GOOD;

      return result;
    }
  }

  return boost::none;
}

void ScoreRelocaliser::reset()
{
  m_exampleReservoirs->clear();
  m_predictionsBlock->Clear();

  m_lastFeaturesAddedStartIdx = 0;
  m_reservoirUpdateStartIdx = 0;
}

void ScoreRelocaliser::update()
{
  // We are back to the first reservoir that was updated when
  // the last batch of features were added to the forest.
  // No need to perform further updates, we would get the same modes.
  // This check works only if the m_maxReservoirsToUpdate quantity
  // remains constant throughout the whole program.
  if (m_reservoirUpdateStartIdx == m_lastFeaturesAddedStartIdx) return;

  const uint32_t updateCount = compute_nb_reservoirs_to_update();
  m_exampleClusterer->find_modes(m_exampleReservoirs->get_reservoirs(),
                                 m_exampleReservoirs->get_reservoirs_size(),
                                 m_predictionsBlock,
                                 m_reservoirUpdateStartIdx,
                                 updateCount);

  update_reservoir_start_idx();
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

uint32_t ScoreRelocaliser::compute_nb_reservoirs_to_update() const
{
  // Either the standard number of reservoirs to update or the remaining group until the end of the memory block.
  return std::min(m_maxReservoirsToUpdate, m_reservoirsCount - m_reservoirUpdateStartIdx);
}

void ScoreRelocaliser::update_reservoir_start_idx()
{
  m_reservoirUpdateStartIdx += m_maxReservoirsToUpdate;

  // Restart from the first reservoir.
  if (m_reservoirUpdateStartIdx >= m_reservoirsCount) m_reservoirUpdateStartIdx = 0;
}

} // namespace grove
