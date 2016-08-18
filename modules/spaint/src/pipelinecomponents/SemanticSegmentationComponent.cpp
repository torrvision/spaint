/**
 * spaint: SemanticSegmentationComponent.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/SemanticSegmentationComponent.h"

#include <rafl/examples/Example.h>
using namespace rafl;

#include "randomforest/ForestUtil.h"
#include "sampling/VoxelSamplerFactory.h"
#include "util/LabelManager.h"
#include "util/MemoryBlockFactory.h"

#ifdef WITH_OPENCV
#include "ocv/OpenCVUtil.h"
#endif

#define DEBUGGING 1

namespace spaint {

//#################### CONSTRUCTORS ####################

SemanticSegmentationComponent::SemanticSegmentationComponent(const Vector2i& depthImageSize, unsigned int seed,
                                                             const Settings_CPtr& settings, size_t maxLabelCount)
: m_maxLabelCount(maxLabelCount)
{
  // Set the maximum numbers of voxels to use for training and prediction.
  // FIXME: These values shouldn't be hard-coded here ultimately.
#ifndef USE_LOW_POWER_MODE
  m_maxPredictionVoxelCount = 8192;
#else
  m_maxPredictionVoxelCount = 512;
#endif
  m_maxTrainingVoxelsPerLabel = 128;

  // Set up the voxel samplers.
  const int raycastResultSize = depthImageSize.width * depthImageSize.height;
  m_predictionSampler = VoxelSamplerFactory::make_uniform_sampler(raycastResultSize, seed, settings->deviceType);
  m_trainingSampler = VoxelSamplerFactory::make_per_label_sampler(maxLabelCount, m_maxTrainingVoxelsPerLabel, raycastResultSize, seed, settings->deviceType);

  // Set up the memory blocks.
  MemoryBlockFactory& mbf = MemoryBlockFactory::instance();
  m_predictionLabelsMB = mbf.make_block<SpaintVoxel::PackedLabel>(m_maxPredictionVoxelCount);
  m_predictionVoxelLocationsMB = mbf.make_block<Vector3s>(m_maxPredictionVoxelCount);
  m_trainingLabelMaskMB = mbf.make_block<bool>(maxLabelCount);
  m_trainingVoxelCountsMB = mbf.make_block<unsigned int>(maxLabelCount);
  m_trainingVoxelLocationsMB = mbf.make_block<Vector3s>(get_max_training_voxel_count());
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

size_t SemanticSegmentationComponent::get_max_prediction_voxel_count() const
{
  return m_maxPredictionVoxelCount;
}

size_t SemanticSegmentationComponent::get_max_training_voxel_count() const
{
  return m_maxLabelCount * m_maxTrainingVoxelsPerLabel;
}

void SemanticSegmentationComponent::run_feature_inspection(SemanticSegmentationModel& model, const RenderState_CPtr& renderState)
{
  // Get the voxels (if any) selected by the user (prior to selection transformation).
  Selector::Selection_CPtr selection = model.get_selector()->get_selection();

  // If the user hasn't selected a single voxel, early out.
  if(!selection || selection->dataSize != 1) return;

  // Calculate the feature descriptor for the selected voxel.
  boost::shared_ptr<ORUtils::MemoryBlock<float> > featuresMB = MemoryBlockFactory::instance().make_block<float>(model.get_feature_calculator()->get_feature_count());
  model.get_feature_calculator()->calculate_features(*selection, model.get_scene().get(), *featuresMB);

#ifdef WITH_OPENCV
  // Convert the feature descriptor into an OpenCV image and show it in a window.
  featuresMB->UpdateHostFromDevice();
  const float *features = featuresMB->GetData(MEMORYDEVICE_CPU);
  const int patchSize = static_cast<int>(model.get_patch_size());
  cv::Mat3b featureInspectionImage = OpenCVUtil::make_rgb_image(features, patchSize, patchSize);

  const float scaleFactor = 10.0f;
  cv::resize(featureInspectionImage, featureInspectionImage, cv::Size(), scaleFactor, scaleFactor, CV_INTER_NN);

  cv::imshow("Feature Inspection", featureInspectionImage);
  const int delayMs = 1;
  cv::waitKey(delayMs);  // this is required in order to make OpenCV actually show the window
#endif
}

void SemanticSegmentationComponent::run_prediction(SemanticSegmentationModel& model, const RenderState_CPtr& samplingRenderState)
{
  // If we haven't been provided with a camera position from which to sample, early out.
  if(!samplingRenderState) return;

  // If the random forest is not yet valid, early out.
  if(!model.get_forest()->is_valid()) return;

  // Sample some voxels for which to predict labels.
  m_predictionSampler->sample_voxels(samplingRenderState->raycastResult, m_maxPredictionVoxelCount, *m_predictionVoxelLocationsMB);

  // Calculate feature descriptors for the sampled voxels.
  model.get_feature_calculator()->calculate_features(*m_predictionVoxelLocationsMB, model.get_scene().get(), *model.get_prediction_features());
  std::vector<Descriptor_CPtr> descriptors = ForestUtil::make_descriptors(*model.get_prediction_features(), m_maxPredictionVoxelCount, model.get_feature_calculator()->get_feature_count());

  // Predict labels for the voxels based on the feature descriptors.
  SpaintVoxel::PackedLabel *labels = m_predictionLabelsMB->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < static_cast<int>(m_maxPredictionVoxelCount); ++i)
  {
    labels[i] = SpaintVoxel::PackedLabel(model.get_forest()->predict(descriptors[i]), SpaintVoxel::LG_FOREST);
  }

  m_predictionLabelsMB->UpdateDeviceFromHost();

  // Mark the voxels with their predicted labels.
  model.get_voxel_marker()->mark_voxels(*m_predictionVoxelLocationsMB, *m_predictionLabelsMB, model.get_scene().get());
}

void SemanticSegmentationComponent::run_training(SemanticSegmentationModel& model, const RenderState_CPtr& samplingRenderState)
{
  // If we haven't been provided with a camera position from which to sample, early out.
  if(!samplingRenderState) return;

  // Calculate a mask indicating the labels that are currently in use and from which we want to train.
  // Note that we deliberately avoid training from the background label (0), since the entire scene is
  // initially labelled as background and so training from the background would cause us to learn
  // incorrect labels for non-background things.
  LabelManager_CPtr labelManager = model.get_label_manager();
  const size_t maxLabelCount = labelManager->get_max_label_count();
  bool *labelMask = m_trainingLabelMaskMB->GetData(MEMORYDEVICE_CPU);
  labelMask[0] = false;
  for(size_t i = 1; i < maxLabelCount; ++i)
  {
    labelMask[i] = labelManager->has_label(static_cast<SpaintVoxel::Label>(i));
  }
  m_trainingLabelMaskMB->UpdateDeviceFromHost();

  // Sample voxels from the scene to use for training the random forest.
  const ORUtils::Image<Vector4f> *raycastResult = samplingRenderState->raycastResult;
  m_trainingSampler->sample_voxels(raycastResult, model.get_scene().get(), *m_trainingLabelMaskMB, *m_trainingVoxelLocationsMB, *m_trainingVoxelCountsMB);

#if DEBUGGING
  // Output the numbers of voxels sampled for each label (for debugging purposes).
  for(size_t i = 0; i < m_trainingVoxelCountsMB->dataSize; ++i)
  {
    std::cout << m_trainingVoxelCountsMB->GetData(MEMORYDEVICE_CPU)[i] << ' ';
  }
  std::cout << '\n';

  // Make sure that the sampled voxels are available on the CPU so that they can be checked.
  m_trainingVoxelLocationsMB->UpdateHostFromDevice();
#endif

  // Compute feature vectors for the sampled voxels.
  model.get_feature_calculator()->calculate_features(*m_trainingVoxelLocationsMB, model.get_scene().get(), *model.get_training_features());

  // Make the training examples.
  typedef boost::shared_ptr<const Example<SpaintVoxel::Label> > Example_CPtr;
  std::vector<Example_CPtr> examples = ForestUtil::make_examples<SpaintVoxel::Label>(
    *model.get_training_features(),
    *m_trainingVoxelCountsMB,
    model.get_feature_calculator()->get_feature_count(),
    m_maxTrainingVoxelsPerLabel,
    maxLabelCount
  );

  // Train the forest.
  const size_t splitBudget = 20;
  model.get_forest()->add_examples(examples);
  model.get_forest()->train(splitBudget);
}

}
