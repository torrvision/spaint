/**
 * grove: ScoreRelocaliser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "relocalisation/interface/ScoreRelocaliser.h"

#include <spaint/util/MemoryBlockFactory.h>
using spaint::MemoryBlockFactory;

#include "clustering/ExampleClustererFactory.h"
#include "features/FeatureCalculatorFactory.h"
#include "forests/DecisionForestFactory.h"
#include "ransac/RansacFactory.h"
#include "reservoirs/ExampleReservoirsFactory.h"

namespace grove {

ScoreRelocaliser::ScoreRelocaliser(ITMLib::ITMLibSettings::DeviceType deviceType, const std::string &forestFilename)
{
  m_featureCalculator = FeatureCalculatorFactory::make_da_rgbd_patch_feature_calculator(deviceType);
  m_scoreForest = DecisionForestFactory<DescriptorType, TREE_COUNT>::make_forest(deviceType, forestFilename);

  m_reservoirsCount = m_scoreForest->get_nb_leaves();
  const uint32_t reservoirCapacity = 1024;
  const uint32_t rngSeed = 42;
  m_exampleReservoirs = ExampleReservoirsFactory<ExampleType>::make_reservoirs(deviceType, reservoirCapacity, m_reservoirsCount, rngSeed);

  // Tentative values
  const float clustererSigma = 0.1f;
  const float clustererTau = 0.05f;
  const uint32_t maxClusterCount = Prediction3DColour::MAX_MODES;
  const uint32_t minClusterSize = 20;
  m_exampleClusterer = ExampleClustererFactory<ExampleType, ClusterType>::make_clusterer(deviceType, clustererSigma, clustererTau, maxClusterCount, minClusterSize);

  // CUDA implementation is slow, force CPU for now. TODO: fix.
//  m_preemptiveRansac = RansacFactory::make_preemptive_ransac(deviceType);
  m_preemptiveRansac = RansacFactory::make_preemptive_ransac(ITMLib::ITMLibSettings::DEVICE_CPU);

  // Setup memory blocks/images
  MemoryBlockFactory &mbf = MemoryBlockFactory::instance();
  m_predictionsBlock = mbf.make_block<ClusterType>(m_reservoirsCount);
  m_predictionsImage = mbf.make_image<ClusterType>();
  m_rgbdPatchDescriptorImage = mbf.make_image<DescriptorType>();
  m_rgbdPatchKeypointsImage = mbf.make_image<ExampleType>();
  m_leafIndicesImage = mbf.make_image<LeafIndices>();

  m_maxReservoirsToUpdate = 256;

  // Clear state
  reset();
}

void ScoreRelocaliser::reset()
{
  m_exampleReservoirs->clear();
  m_predictionsBlock->Clear();

  m_lastFeaturesAddedStartIdx = 0;
  m_reservoirUpdateStartIdx = 0;
}

void ScoreRelocaliser::integrate_measurements(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage, const Vector4f &depthIntrinsics, const ORUtils::SE3Pose &cameraPose)
{
  // First: select keypoints and compute descriptors.
  compute_features(colourImage, depthImage, depthIntrinsics, cameraPose.GetInvM());

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

void ScoreRelocaliser::compute_features(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage, const Vector4f &depthIntrinsics, const Matrix4f &invCameraPose) const
{
  m_featureCalculator->compute_keypoints_and_features(colourImage, depthImage, invCameraPose,
                                                      depthIntrinsics, m_rgbdPatchKeypointsImage.get(), m_rgbdPatchDescriptorImage.get());
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
