/**
 * grove: ScoreRelocaliser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "relocalisation/interface/ScoreRelocaliser.h"
using namespace ORUtils;
using namespace tvgutil;

#include <boost/filesystem.hpp>
namespace bf = boost::filesystem;

#include <orx/base/MemoryBlockFactory.h>
using namespace orx;

#include "clustering/ExampleClustererFactory.h"
#include "features/FeatureCalculatorFactory.h"
#include "ransac/PreemptiveRansacFactory.h"
#include "relocalisation/shared/ScoreGTRelocaliser_Shared.h"

namespace grove {

//#################### CONSTRUCTORS ####################

ScoreRelocaliser::ScoreRelocaliser(const SettingsContainer_CPtr& settings, const std::string& settingsNamespace, DeviceType deviceType)
: m_backed(false),
  m_deviceType(deviceType),
  m_maxX(static_cast<float>(INT_MIN)),
  m_maxY(static_cast<float>(INT_MIN)),
  m_maxZ(static_cast<float>(INT_MIN)),
  m_minX(static_cast<float>(INT_MAX)),
  m_minY(static_cast<float>(INT_MAX)),
  m_minZ(static_cast<float>(INT_MAX)),
  m_settings(settings),
  m_settingsNamespace(settingsNamespace)
{
  // Determine the top-level parameters for the relocaliser.
  m_enableDebugging = m_settings->get_first_value<bool>(settingsNamespace + "enableDebugging", false);
  m_maxRelocalisationsToOutput = m_settings->get_first_value<uint32_t>(settingsNamespace + "maxRelocalisationsToOutput", 1);

  // Determine the reservoir-related parameters.
  m_maxReservoirsToUpdate = m_settings->get_first_value<uint32_t>(settingsNamespace + "maxReservoirsToUpdate", 256);  // Update the modes associated with this number of reservoirs for each train/update call.
  m_reservoirCapacity = m_settings->get_first_value<uint32_t>(settingsNamespace + "reservoirCapacity", 1024);
  m_rngSeed = m_settings->get_first_value<uint32_t>(settingsNamespace + "rngSeed", 42);

  // Determine the clustering-related parameters (the defaults are tentative values that seem to work).
  m_clustererSigma = m_settings->get_first_value<float>(settingsNamespace + "clustererSigma", 0.1f);
  m_clustererTau = m_settings->get_first_value<float>(settingsNamespace + "clustererTau", 0.05f);
  m_maxClusterCount = m_settings->get_first_value<uint32_t>(settingsNamespace + "maxClusterCount", ScorePrediction::Capacity);
  m_minClusterSize = m_settings->get_first_value<uint32_t>(settingsNamespace + "minClusterSize", 20);

  // Check that the maximum number of clusters to store in each leaf is within range.
  if(m_maxClusterCount > ScorePrediction::Capacity)
  {
    throw std::invalid_argument(settingsNamespace + "maxClusterCount > ScorePrediction::Capacity");
  }

  // Allocate the internal images.
  MemoryBlockFactory& mbf = MemoryBlockFactory::instance();
  m_descriptorsImage = mbf.make_image<DescriptorType>();
  m_groundTruthPredictionsImage = mbf.make_image<ScorePrediction>();
  m_keypointsImage = mbf.make_image<ExampleType>();
  m_predictionsImage = mbf.make_image<ScorePrediction>();

  // Instantiate the sub-components.
  m_featureCalculator = FeatureCalculatorFactory::make_da_rgbd_patch_feature_calculator(deviceType);
  m_preemptiveRansac = PreemptiveRansacFactory::make_preemptive_ransac(settings, settingsNamespace + "PreemptiveRansac.", deviceType);
}

//#################### DESTRUCTOR ####################

ScoreRelocaliser::~ScoreRelocaliser() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void ScoreRelocaliser::finish_training()
{
  // If this relocaliser is "backed" by another one, early out.
  if(m_backed) return;

  boost::lock_guard<boost::recursive_mutex> lock(m_mutex);

  // First update all of the clusters.
  update_all_clusters();

  // Then kill the contents of the reservoirs (we won't need them any more).
  m_relocaliserState->exampleReservoirs.reset();
  m_relocaliserState->lastExamplesAddedStartIdx = 0;
  m_relocaliserState->reservoirUpdateStartIdx = 0;

  // Finally, release the example clusterer.
  m_exampleClusterer.reset();
}

void ScoreRelocaliser::get_best_poses(std::vector<PoseCandidate>& poseCandidates) const
{
  m_preemptiveRansac->get_best_poses(poseCandidates);
}

Keypoint3DColourImage_CPtr ScoreRelocaliser::get_keypoints_image() const
{
  return m_keypointsImage;
}

ScorePredictionsImage_CPtr ScoreRelocaliser::get_predictions_image() const
{
  return m_predictionsImage;
}

ORUChar4Image_CPtr ScoreRelocaliser::get_visualisation_image(const std::string& key) const
{
  if(key == "gtpoints") return m_groundTruthPixelsToPointsImage;
  else if(key == "points") return m_pixelsToPointsImage;
  else return ORUChar4Image_CPtr();
}

void ScoreRelocaliser::load_from_disk(const std::string& inputFolder)
{
  // If this relocaliser is "backed" by another one, early out.
  if(m_backed) return;

  // Otherwise, load its internal state from disk.
  m_relocaliserState->load_from_disk(inputFolder);
}

std::vector<Relocaliser::Result> ScoreRelocaliser::relocalise(const ORUChar4Image *colourImage, const ORFloatImage *depthImage, const Vector4f& depthIntrinsics) const
{
  boost::lock_guard<boost::recursive_mutex> lock(m_mutex);

  std::vector<Result> results;

  // Iff we have enough valid depth values, try to estimate the camera pose:
  if(m_preemptiveRansac->count_valid_depths(depthImage) > m_preemptiveRansac->get_min_nb_required_points())
  {
    // Step 1: Extract keypoints from the RGB-D image and compute descriptors for them.
    // FIXME: We only need to compute the descriptors if we're using the forest.
    m_featureCalculator->compute_keypoints_and_features(colourImage, depthImage, depthIntrinsics, m_keypointsImage.get(), m_descriptorsImage.get());

    // Step 2: Create a single SCoRe prediction (a single set of clusters) for each keypoint.
    make_predictions(colourImage);

    // Step 3: Perform P-RANSAC to try to estimate the camera pose.
    boost::optional<PoseCandidate> poseCandidate = m_preemptiveRansac->estimate_pose(m_keypointsImage, m_predictionsImage);

    // Step 4: If we succeeded in estimating a camera pose:
    if(poseCandidate)
    {
      // Add the pose to the results.
      Result result;
      result.pose.SetInvM(poseCandidate->cameraPose);
      result.quality = RELOCALISATION_GOOD;
      result.score = poseCandidate->energy;
      results.push_back(result);

      // If we're outputting multiple poses:
      if(m_maxRelocalisationsToOutput > 1)
      {
        // Get all of the candidates that survived the initial culling process during P-RANSAC.
        std::vector<PoseCandidate> candidates;
        m_preemptiveRansac->get_best_poses(candidates);

        // Add the best candidates to the results (skipping the first one, since it's the same one returned by estimate_pose above).
        const size_t maxElements = std::min<size_t>(candidates.size(), m_maxRelocalisationsToOutput);
        for(size_t i = 1; i < maxElements; ++i)
        {
          Result result;
          result.pose.SetInvM(candidates[i].cameraPose);
          result.quality = RELOCALISATION_GOOD;
          result.score = candidates[i].energy;
          results.push_back(result);
        }
      }
    }
  }

  // If debugging is enabled, update the visualisation images.
  if(m_enableDebugging)
  {
    make_visualisation_images(depthImage, results);
  }

  // If we're using the ground truth camera trajectory, increment the ground truth frame index.
  if(m_groundTruthTrajectory && m_groundTruthFrameIndex < m_groundTruthTrajectory->size())
  {
    ++m_groundTruthFrameIndex;
  }

  return results;
}

void ScoreRelocaliser::reset()
{
  // If this relocaliser is "backed" by another one, early out.
  if(m_backed) return;

  boost::lock_guard<boost::recursive_mutex> lock(m_mutex);

  // Set up the clusterer if it isn't currently allocated (note that it can be deallocated by finish_training, so this can't just be moved to the constructor).
  if(!m_exampleClusterer)
  {
    m_exampleClusterer = ExampleClustererFactory<ExampleType,ClusterType,PredictionType::Capacity>::make_clusterer(
      m_clustererSigma, m_clustererTau, m_maxClusterCount, m_minClusterSize, m_deviceType
    );
  }

  // If the relocaliser's state already exists, reset it; if not, allocate it.
  if(m_relocaliserState) m_relocaliserState->reset();
  else m_relocaliserState.reset(new ScoreRelocaliserState(m_reservoirCount, m_reservoirCapacity, m_deviceType, m_rngSeed));

  // Reset the ground truth frame index.
  m_groundTruthFrameIndex = 0;
}

void ScoreRelocaliser::save_to_disk(const std::string& outputFolder) const
{
  // If this relocaliser is "backed" by another one, early out.
  if(m_backed) return;

  // First make sure that the output folder exists.
  bf::create_directories(outputFolder);

  // Then save the relocaliser's internal state to disk.
  m_relocaliserState->save_to_disk(outputFolder);
}

void ScoreRelocaliser::set_backing_relocaliser(const ScoreRelocaliser_Ptr& backingRelocaliser)
{
  m_relocaliserState = backingRelocaliser->m_relocaliserState;
  m_backed = true;
}

void ScoreRelocaliser::set_ground_truth_trajectory(const std::vector<ORUtils::SE3Pose>& groundTruthTrajectory)
{
  m_groundTruthTrajectory = groundTruthTrajectory;
}

void ScoreRelocaliser::train(const ORUChar4Image *colourImage, const ORFloatImage *depthImage,
                             const Vector4f& depthIntrinsics, const ORUtils::SE3Pose& cameraPose)
{
  boost::lock_guard<boost::recursive_mutex> lock(m_mutex);

  // If debugging is enabled, update the maximum and minimum x, y and z coordinates visited by the camera during training.
  if(m_enableDebugging)
  {
    m_maxX = std::max(m_maxX, cameraPose.GetT().x);
    m_maxY = std::max(m_maxY, cameraPose.GetT().y);
    m_maxZ = std::max(m_maxZ, cameraPose.GetT().z);
    m_minX = std::min(m_minX, cameraPose.GetT().x);
    m_minY = std::min(m_minY, cameraPose.GetT().y);
    m_minZ = std::min(m_minZ, cameraPose.GetT().z);
  }

  // If this relocaliser is "backed" by another one, early out.
  if(m_backed) return;

  // If we haven't reset since the last time finish_training was called, throw.
  if(!m_relocaliserState->exampleReservoirs)
  {
    throw std::runtime_error("Error: finish_training() has been called; the relocaliser cannot be trained again until reset() is called");
  }

  // Call the hook function (a function that should be overridden by derived classes to perform the actual training).
  train_sub(colourImage, depthImage, depthIntrinsics, cameraPose);

  // If there are any reservoirs:
  if(m_reservoirCount > 0)
  {
    // Cluster some of the reservoirs.
    const uint32_t nbReservoirsToUpdate = compute_nb_reservoirs_to_update();
    m_exampleClusterer->cluster_examples(
      m_relocaliserState->exampleReservoirs->get_reservoirs(), m_relocaliserState->exampleReservoirs->get_reservoir_sizes(),
      m_relocaliserState->reservoirUpdateStartIdx, nbReservoirsToUpdate, m_relocaliserState->predictionsBlock
    );

    // Store the index of the first reservoir that was just updated so that we can tell when there are no more clusters to update.
    m_relocaliserState->lastExamplesAddedStartIdx = m_relocaliserState->reservoirUpdateStartIdx;

    // Update the index of the first reservoir to subject to clustering during the next train/update call.
    update_reservoir_start_idx();
  }
}

void ScoreRelocaliser::update()
{
  // If this relocaliser is "backed" by another one, early out.
  if(m_backed) return;

  boost::lock_guard<boost::recursive_mutex> lock(m_mutex);

  if(!m_relocaliserState->exampleReservoirs)
  {
    throw std::runtime_error("Error: finish_training() has been called; the relocaliser cannot be updated again until reset() is called");
  }

  // If we are back to the first reservoir that was updated when the last batch of examples were added to the
  // reservoirs, there is no need to perform further updates, since we would get the same clusters. Note that
  // this check only works if m_maxReservoirsToUpdate remains constant throughout the whole program.
  if(m_relocaliserState->reservoirUpdateStartIdx == m_relocaliserState->lastExamplesAddedStartIdx) return;

  // Otherwise, cluster the next batch of reservoirs, and update the index of the first reservoir to subject to
  // clustering during the next train/update call.
  const uint32_t updateCount = compute_nb_reservoirs_to_update();
  m_exampleClusterer->cluster_examples(
    m_relocaliserState->exampleReservoirs->get_reservoirs(), m_relocaliserState->exampleReservoirs->get_reservoir_sizes(),
    m_relocaliserState->reservoirUpdateStartIdx, updateCount, m_relocaliserState->predictionsBlock
  );

  update_reservoir_start_idx();
}

void ScoreRelocaliser::update_all_clusters()
{
  // If this relocaliser is "backed" by another one, early out.
  if(m_backed) return;

  boost::lock_guard<boost::recursive_mutex> lock(m_mutex);

  // Repeatedly call update until we get back to the batch of reservoirs that was updated last time train() was called.
  while(m_relocaliserState->reservoirUpdateStartIdx != m_relocaliserState->lastExamplesAddedStartIdx)
  {
    update();
  }
}

//#################### PROTECTED MEMBER FUNCTIONS ####################

void ScoreRelocaliser::make_visualisation_images(const ORFloatImage *depthImage, const std::vector<Result>& results) const
{
  if(m_groundTruthTrajectory && m_groundTruthFrameIndex < m_groundTruthTrajectory->size())
  {
    // Update the normal pixel to points image, using the ground truth world to camera transformation.
    const Matrix4f& worldToCamera = (*m_groundTruthTrajectory)[m_groundTruthFrameIndex].GetM();
    update_pixels_to_points_image(worldToCamera, m_predictionsImage, m_pixelsToPointsImage);

    // If requested, also update the ground truth pixel to points image.
    if(m_settings->get_first_value<bool>(m_settingsNamespace + "makeGroundTruthPointsImage", false))
    {
      const Matrix4f& cameraToWorld = (*m_groundTruthTrajectory)[m_groundTruthFrameIndex].GetInvM();
      set_ground_truth_predictions_for_keypoints(m_keypointsImage, cameraToWorld, m_groundTruthPredictionsImage);
      update_pixels_to_points_image(worldToCamera, m_groundTruthPredictionsImage, m_groundTruthPixelsToPointsImage);
    }
  }
  else if(!results.empty())
  {
    // Update the normal pixel to points image, using the "best" relocalised pose as the world to camera transformation.
    // Note: We use this pose as a default, even though it may later be either refined by ICP or discarded in favour of a different pose.
    update_pixels_to_points_image(results[0].pose, m_predictionsImage, m_pixelsToPointsImage);
  }
}

void ScoreRelocaliser::set_ground_truth_predictions_for_keypoints(const Keypoint3DColourImage_CPtr& keypointsImage, const Matrix4f& cameraToWorld,
                                                                  ScorePredictionsImage_Ptr& outputPredictions) const
{
  const Vector2i imgSize = keypointsImage->noDims;

  // Make sure that the output predictions image has the right size (this is a no-op after the first time).
  outputPredictions->ChangeDims(imgSize);

  // If necessary, copy the keypoints image across to the CPU.
  if(m_deviceType == DEVICE_CUDA) keypointsImage->UpdateHostFromDevice();

  const Keypoint3DColour *keypointsPtr = keypointsImage->GetData(MEMORYDEVICE_CPU);
  ScorePrediction *outputPredictionsPtr = outputPredictions->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int y = 0; y < imgSize.y; ++y)
  {
    for(int x = 0; x < imgSize.x; ++x)
    {
      set_ground_truth_prediction_for_keypoint(x, y, imgSize, keypointsPtr, cameraToWorld, outputPredictionsPtr);
    }
  }

  // If necessary, copy the output predictions back across to the GPU.
  if(m_deviceType == DEVICE_CUDA) outputPredictions->UpdateDeviceFromHost();
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

uint32_t ScoreRelocaliser::compute_nb_reservoirs_to_update() const
{
  // Either the standard number of reservoirs to update, or the number remaining before the end of the memory block.
  return std::min(m_maxReservoirsToUpdate, m_reservoirCount - m_relocaliserState->reservoirUpdateStartIdx);
}

void ScoreRelocaliser::update_pixels_to_points_image(const ORUtils::SE3Pose& worldToCamera, const ScorePredictionsImage_Ptr& predictionsImage,
                                                     ORUChar4Image_Ptr& pixelsToPointsImage) const
{
  // Ensure that the keypoints and SCoRe predictions are available on the CPU.
  m_keypointsImage->UpdateHostFromDevice();
  predictionsImage->UpdateHostFromDevice();

  // If the pixels to points image hasn't been allocated yet, allocate it now.
  if(!pixelsToPointsImage) pixelsToPointsImage.reset(new ORUChar4Image(m_keypointsImage->noDims, true, true));

  // For each pixel:
  Vector4u *p = pixelsToPointsImage->GetData(MEMORYDEVICE_CPU);
  for(int i = 0, pixelCount = static_cast<int>(pixelsToPointsImage->dataSize); i < pixelCount; ++i, ++p)
  {
    p->r = p->g = p->b = 0;
    p->a = 255;

    // If the pixel has a valid keypoint, look up the position of the cluster (if any) in the corresponding prediction that is closest to it.
    const ExampleType& keypoint = m_keypointsImage->GetData(MEMORYDEVICE_CPU)[i];
    if(!keypoint.valid) continue;
    const PredictionType& prediction = predictionsImage->GetData(MEMORYDEVICE_CPU)[i];
    const int closestModeIdx = find_closest_mode(worldToCamera.GetInvM() * keypoint.position, prediction);
    if(closestModeIdx == -1) continue;
    const Vector3f& clusterPos = prediction.elts[closestModeIdx].position;

    // Colour the pixel in the pixels to points image based on the cluster's position in world space.
    float scale = 2.0f;
    float centre = 0.5f;
    float offset = centre - 1.0f / (scale * 2.0f);
    float xCoeff = ((CLAMP(clusterPos.x,m_minX,m_maxX) - m_minX) / (m_maxX - m_minX)) / scale + offset;
    float yCoeff = ((CLAMP(clusterPos.y,m_minY,m_maxY) - m_minY) / (m_maxY - m_minY)) / scale + offset;
    float zCoeff = ((CLAMP(clusterPos.z,m_minZ,m_maxZ) - m_minZ) / (m_maxZ - m_minZ)) / scale + offset;
    p->r = static_cast<unsigned char>(255 * xCoeff);
    p->g = static_cast<unsigned char>(255 * yCoeff);
    p->b = static_cast<unsigned char>(255 * zCoeff);
    p->a = 255;
  }

  // Update the GPU copy of the pixels to points image (if it exists).
  pixelsToPointsImage->UpdateDeviceFromHost();
}

void ScoreRelocaliser::update_reservoir_start_idx()
{
  m_relocaliserState->reservoirUpdateStartIdx += m_maxReservoirsToUpdate;

  // If we go past the end of the list of reservoirs, loop back round.
  if(m_relocaliserState->reservoirUpdateStartIdx >= m_reservoirCount)
  {
    m_relocaliserState->reservoirUpdateStartIdx = 0;
  }
}

}
