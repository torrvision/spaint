/**
 * grove: ScoreRelocaliser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "relocalisation/interface/ScoreRelocaliser.h"

#include <itmx/MemoryBlockFactory.h>
using namespace itmx;

namespace grove {

ScoreRelocaliser::ScoreRelocaliser(const std::string &forestFilename)
{
  // Just set the variables, instantiation of the sub-algorithms is left to the sub class.
  m_forestFilename = forestFilename;

  // except m_reservoirsCount since that number depends on the forest.
  m_reservoirsCapacity = 1024;
  m_rngSeed = 42;

  // Tentative values
  m_clustererSigma = 0.1f;
  m_clustererTau = 0.05f;
  m_maxClusterCount = Prediction3DColour::MAX_MODES;
  m_minClusterSize = 20;

  // Setup memory blocks/images (except m_predictionsBlock since its size depends on the forest)
  MemoryBlockFactory &mbf = MemoryBlockFactory::instance();
  m_predictionsImage = mbf.make_image<ClusterType>();
  m_rgbdPatchDescriptorImage = mbf.make_image<DescriptorType>();
  m_rgbdPatchKeypointsImage = mbf.make_image<ExampleType>();
  m_leafIndicesImage = mbf.make_image<LeafIndices>();

  m_maxReservoirsToUpdate = 256;
}

ScoreRelocaliser::~ScoreRelocaliser()
{}

void ScoreRelocaliser::reset()
{
  m_exampleReservoirs->clear();
  m_predictionsBlock->Clear();

  m_lastFeaturesAddedStartIdx = 0;
  m_reservoirUpdateStartIdx = 0;
}

void ScoreRelocaliser::integrate_measurements(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage, const Vector4f &depthIntrinsics, const Matrix4f &cameraPose)
{
  // First: select keypoints and compute descriptors.
  m_featureCalculator->compute_keypoints_and_features(colourImage, depthImage, cameraPose, depthIntrinsics, m_rgbdPatchKeypointsImage.get(), m_rgbdPatchDescriptorImage.get());

  // Second: find the leaves associated to the keypoints.
  m_scoreForest->find_leaves(m_rgbdPatchDescriptorImage, m_leafIndicesImage);

  // Third: add keypoints to the correct reservoirs.
  m_exampleReservoirs->add_examples(m_rgbdPatchKeypointsImage, m_leafIndicesImage);

  // Fourth: cluster some of the reservoirs.
  const uint32_t updateCount = compute_nb_reservoirs_to_update();
  m_exampleClusterer->find_modes(m_exampleReservoirs->get_reservoirs(), m_exampleReservoirs->get_reservoirs_size(), m_predictionsBlock, m_reservoirUpdateStartIdx, updateCount);

  // Fifth: save the current index to indicate that reservoirs up to such index have to be clustered to reflect the examples added.
  m_lastFeaturesAddedStartIdx = m_reservoirUpdateStartIdx;

  // Finally: update starting index for the next invocation of either this function or idle_update().
  update_reservoir_start_idx();
}

void ScoreRelocaliser::idle_update()
{
  // We are back to the first reservoir that was updated when
  // the last batch of features were added to the forest.
  // No need to update further, we would get the same modes.
  // This check works only if the m_maxReservoirsToUpdate
  // remains constant throughout the whole program.
  if (m_reservoirUpdateStartIdx == m_lastFeaturesAddedStartIdx)
    return;

  const uint32_t updateCount = compute_nb_reservoirs_to_update();
  m_exampleClusterer->find_modes(m_exampleReservoirs->get_reservoirs(), m_exampleReservoirs->get_reservoirs_size(), m_predictionsBlock, m_reservoirUpdateStartIdx, updateCount);

  update_reservoir_start_idx();
}

boost::optional<PoseCandidate> ScoreRelocaliser::estimate_pose(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage, const Vector4f &depthIntrinsics)
{
  boost::optional<PoseCandidate> result;

  // Try to estimate a pose only if we have enough depth values.
  if(m_lowLevelEngine->CountValidDepths(depthImage) > m_preemptiveRansac->get_min_nb_required_points())
  {
    // First: select keypoints and compute descriptors.
    m_featureCalculator->compute_keypoints_and_features(colourImage, depthImage, depthIntrinsics, m_rgbdPatchKeypointsImage.get(), m_rgbdPatchDescriptorImage.get());

    // Second: find the leaves associated to the keypoints.
    m_scoreForest->find_leaves(m_rgbdPatchDescriptorImage, m_leafIndicesImage);

    // Third: get the predictions associated to those leaves.
    get_predictions_for_leaves(m_leafIndicesImage, m_predictionsBlock, m_predictionsImage);

    // Finally: perform RANSAC.
    result = m_preemptiveRansac->estimate_pose(m_rgbdPatchKeypointsImage, m_predictionsImage);
  }

  return result;
}

uint32_t ScoreRelocaliser::compute_nb_reservoirs_to_update() const
{
  return std::min<int>(m_maxReservoirsToUpdate, m_reservoirsCount - m_reservoirUpdateStartIdx);
}

void ScoreRelocaliser::update_reservoir_start_idx()
{
  m_reservoirUpdateStartIdx += m_maxReservoirsToUpdate;

  if (m_reservoirUpdateStartIdx >= m_reservoirsCount)
    m_reservoirUpdateStartIdx = 0;
}

}
