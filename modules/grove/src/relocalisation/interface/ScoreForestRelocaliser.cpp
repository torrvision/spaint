/**
 * grove: ScoreForestRelocaliser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "relocalisation/interface/ScoreForestRelocaliser.h"
using namespace ORUtils;

#ifdef WITH_OPENCV
#include <opencv2/opencv.hpp>
#endif

#include <orx/base/MemoryBlockFactory.h>
using namespace orx;

#include <tvgutil/filesystem/PathFinder.h>
using namespace tvgutil;

#include "forests/DecisionForestFactory.h"

namespace grove {

//#################### CONSTRUCTORS ####################

ScoreForestRelocaliser::ScoreForestRelocaliser(const SettingsContainer_CPtr& settings, const std::string& settingsNamespace, DeviceType deviceType)
: ScoreRelocaliser(settings, settingsNamespace, deviceType)
{
  // Allocate the internal images.
  MemoryBlockFactory& mbf = MemoryBlockFactory::instance();
  m_leafIndicesImage = mbf.make_image<LeafIndices>();

  // Either construct a random SCoRe forest, or load one from disk.
  const bool randomlyGenerateForest = m_settings->get_first_value<bool>(settingsNamespace + "randomlyGenerateForest", false);
  if(randomlyGenerateForest)
  {
    m_scoreForest = DecisionForestFactory<DescriptorType,FOREST_TREE_COUNT>::make_randomly_generated_forest(m_settings, deviceType);
  }
  else
  {
    const std::string modelFilename = m_settings->get_first_value<std::string>(settingsNamespace + "modelFilename", (find_subdir_from_executable("resources") / "DefaultRelocalisationForest.rf").string());
    std::cout << "Loading relocalisation forest from: " << modelFilename << '\n';
    m_scoreForest = DecisionForestFactory<DescriptorType,FOREST_TREE_COUNT>::make_forest(modelFilename, deviceType);
  }

  // Set the number of reservoirs to allocate to the number of leaves in the forest (i.e. there will be one reservoir per leaf).
  m_reservoirCount = m_scoreForest->get_nb_leaves();

  // Set up the example clusterer and the relocaliser's internal state.
  reset();
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

ScorePrediction ScoreForestRelocaliser::get_prediction(uint32_t treeIdx, uint32_t leafIdx) const
{
  // Ensure that the specified leaf is valid (throw if not).
  ensure_valid_leaf(treeIdx, leafIdx);

  // Look up the prediction associated with the leaf and return it.
  const MemoryDeviceType memoryType = m_deviceType == DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
  return m_relocaliserState->predictionsBlock->GetElement(leafIdx * m_scoreForest->get_nb_trees() + treeIdx, memoryType);
}

std::vector<Keypoint3DColour> ScoreForestRelocaliser::get_reservoir_contents(uint32_t treeIdx, uint32_t leafIdx) const
{
  // Ensure that the specified leaf is valid (throw if not).
  ensure_valid_leaf(treeIdx, leafIdx);

  // Look up the size of the reservoir associated with the leaf.
  const MemoryDeviceType memoryType = m_deviceType == DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
  const uint32_t linearReservoirIdx = leafIdx * m_scoreForest->get_nb_trees() + treeIdx;
  const uint32_t reservoirSize = m_relocaliserState->exampleReservoirs->get_reservoir_sizes()->GetElement(linearReservoirIdx, memoryType);

  // Copy the contents of the reservoir into a suitably-sized buffer and return it.
  if(m_deviceType == DEVICE_CUDA) m_relocaliserState->exampleReservoirs->get_reservoirs()->UpdateHostFromDevice();
  const Keypoint3DColour *reservoirsData = m_relocaliserState->exampleReservoirs->get_reservoirs()->GetData(MEMORYDEVICE_CPU);
  const uint32_t reservoirCapacity = m_relocaliserState->exampleReservoirs->get_reservoir_capacity();

  std::vector<Keypoint3DColour> reservoirContents;
  reservoirContents.reserve(reservoirSize);

  for(uint32_t i = 0; i < reservoirSize; ++i)
  {
    reservoirContents.push_back(reservoirsData[linearReservoirIdx * reservoirCapacity + i]);
  }

  return reservoirContents;
}

ORUChar4Image_CPtr ScoreForestRelocaliser::get_visualisation_image(const std::string& key) const
{
  if(key == "leaves") return m_pixelsToLeavesImage;
  else return ScoreRelocaliser::get_visualisation_image(key);
}

//#################### PROTECTED MEMBER FUNCTIONS ####################

void ScoreForestRelocaliser::ensure_valid_leaf(uint32_t treeIdx, uint32_t leafIdx) const
{
  if(treeIdx >= m_scoreForest->get_nb_trees() || leafIdx >= m_scoreForest->get_nb_leaves_in_tree(treeIdx))
  {
    throw std::invalid_argument("Error: Invalid tree or leaf index");
  }
}

void ScoreForestRelocaliser::make_visualisation_images(const ORFloatImage *depthImage, const std::vector<Result>& results) const
{
  ScoreRelocaliser::make_visualisation_images(depthImage, results);
  update_pixels_to_leaves_image(depthImage);
}

void ScoreForestRelocaliser::make_predictions(const ORUChar4Image *colourImage) const
{
  // Find all of the leaves in the forest that are associated with the descriptors for the keypoints.
  m_scoreForest->find_leaves(m_descriptorsImage, m_leafIndicesImage);

  // Merge the SCoRe predictions (sets of clusters) associated with each keypoint to create a single SCoRe prediction per keypoint.
  merge_predictions_for_keypoints(m_leafIndicesImage, m_predictionsImage);
}

void ScoreForestRelocaliser::train_sub(const ORUChar4Image *colourImage, const ORFloatImage *depthImage,
                                       const Vector4f& depthIntrinsics, const ORUtils::SE3Pose& cameraPose)
{
  // Extract keypoints from the RGB-D image and compute descriptors for them.
  const Matrix4f invCameraPose = cameraPose.GetInvM();
  m_featureCalculator->compute_keypoints_and_features(colourImage, depthImage, invCameraPose, depthIntrinsics, m_keypointsImage.get(), m_descriptorsImage.get());

  // Find all of the leaves in the forest that are associated with the descriptors for the keypoints.
  m_scoreForest->find_leaves(m_descriptorsImage, m_leafIndicesImage);

  // Add the keypoints to the relevant reservoirs.
  m_relocaliserState->exampleReservoirs->add_examples(m_keypointsImage, m_leafIndicesImage);
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void ScoreForestRelocaliser::update_pixels_to_leaves_image(const ORFloatImage *depthImage) const
{
#ifdef WITH_OPENCV
  // Ensure that the depth image and leaf indices are available on the CPU.
  depthImage->UpdateHostFromDevice();
  m_leafIndicesImage->UpdateHostFromDevice();

  // Make a map showing which pixels are in which leaves (for the first tree).
  std::map<int,std::vector<int> > leafToRegionMap;
  for(int i = 0, pixelCount = static_cast<int>(m_leafIndicesImage->dataSize); i < pixelCount; ++i)
  {
    const ORUtils::VectorX<int,FOREST_TREE_COUNT>& elt = m_leafIndicesImage->GetData(MEMORYDEVICE_CPU)[i];
    leafToRegionMap[elt[0]].push_back(i);
  }

  // Make greyscale and colour images showing which pixels are in which leaves (for the first tree).
  cv::Mat1b imageG = cv::Mat1b::zeros(m_leafIndicesImage->noDims.y, m_leafIndicesImage->noDims.x);
  const uint32_t featureStep = m_featureCalculator->get_feature_step();
  for(std::map<int,std::vector<int> >::const_iterator jt = leafToRegionMap.begin(), jend = leafToRegionMap.end(); jt != jend; ++jt)
  {
    for(std::vector<int>::const_iterator kt = jt->second.begin(), kend = jt->second.end(); kt != kend; ++kt)
    {
      int x = *kt % m_leafIndicesImage->noDims.x, y = *kt / m_leafIndicesImage->noDims.x;
      if(depthImage->GetData(MEMORYDEVICE_CPU)[y * featureStep * depthImage->noDims.x + x * featureStep] > 0.0f)
      {
        imageG(y,x) = jt->first % 256;
      }
    }
  }

  cv::Mat3b imageC;
  cv::applyColorMap(imageG, imageC, cv::COLORMAP_HSV);

  // If the pixels to leaves image hasn't been allocated yet, allocate it now.
  if(!m_pixelsToLeavesImage) m_pixelsToLeavesImage.reset(new ORUChar4Image(Vector2i(imageC.cols, imageC.rows), true, true));

  for(int y = 0; y < imageC.rows; ++y)
  {
    for(int x = 0; x < imageC.cols; ++x)
    {
      const int offset = y * imageC.cols + x;
      cv::Vec3b& p = imageC(y, x);

      if(depthImage->GetData(MEMORYDEVICE_CPU)[y * featureStep * depthImage->noDims.x + x * featureStep] <= 0.0f)
      {
        p = cv::Vec3b(0,0,0);
      }

      m_pixelsToLeavesImage->GetData(MEMORYDEVICE_CPU)[offset] = Vector4u(p[2], p[1], p[0], 255);
    }
  }

  // Update the GPU copy of the pixels to leaves image (if it exists).
  m_pixelsToLeavesImage->UpdateDeviceFromHost();
#endif
}

}
