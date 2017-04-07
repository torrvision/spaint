/**
 * spaint: SemanticSegmentationComponent.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/SemanticSegmentationComponent.h"

#include <itmx/MemoryBlockFactory.h>
using itmx::MemoryBlockFactory;

#include <rafl/examples/Example.h>
using namespace rafl;

#include "features/FeatureCalculatorFactory.h"
#include "randomforest/ForestUtil.h"
#include "randomforest/SpaintDecisionFunctionGenerator.h"
#include "sampling/VoxelSamplerFactory.h"

#ifdef WITH_OPENCV
#include "ocv/OpenCVUtil.h"
#endif

#define DEBUGGING 1

namespace spaint {

//#################### CONSTRUCTORS ####################

SemanticSegmentationComponent::SemanticSegmentationComponent(const SemanticSegmentationContext_Ptr& context, const std::string& sceneID, unsigned int seed)
: m_context(context), m_sceneID(sceneID)
{
  // Set the maximum numbers of voxels to use for training and prediction.
  // FIXME: These values shouldn't be hard-coded here ultimately.
#ifndef USE_LOW_POWER_MODE
  m_maxPredictionVoxelCount = 8192;
#else
  m_maxPredictionVoxelCount = 512;
#endif
  m_maxTrainingVoxelsPerLabel = 128;
  const size_t maxLabelCount = context->get_label_manager()->get_max_label_count();
  const size_t maxTrainingVoxelCount = maxLabelCount * m_maxTrainingVoxelsPerLabel;

  // Set up the voxel samplers.
  const Vector2i& depthImageSize = context->get_slam_state(sceneID)->get_depth_image_size();
  const int raycastResultSize = depthImageSize.width * depthImageSize.height;
  const Settings_CPtr& settings = context->get_settings();
  m_predictionSampler = VoxelSamplerFactory::make_uniform_sampler(raycastResultSize, seed, settings->deviceType);
  m_trainingSampler = VoxelSamplerFactory::make_per_label_sampler(maxLabelCount, m_maxTrainingVoxelsPerLabel, raycastResultSize, seed, settings->deviceType);

  // Set up the feature calculator.
  // FIXME: These values shouldn't be hard-coded here ultimately.
  m_patchSize = 13;
  const float patchSpacing = 0.01f / settings->sceneParams.voxelSize; // 10mm = 0.01m (dividing by the voxel size, which is in m, expresses the spacing in voxels)
  const size_t binCount = 36;                                         // 10 degrees per bin

  m_featureCalculator = FeatureCalculatorFactory::make_vop_feature_calculator(
    std::max(m_maxPredictionVoxelCount, maxTrainingVoxelCount),
    m_patchSize, patchSpacing, binCount, settings->deviceType
  );

  // Set up the memory blocks needed for prediction and training.
  MemoryBlockFactory& mbf = MemoryBlockFactory::instance();
  const size_t featureCount = m_featureCalculator->get_feature_count();
  m_predictionFeaturesMB = mbf.make_block<float>(m_maxPredictionVoxelCount * featureCount);
  m_predictionLabelsMB = mbf.make_block<SpaintVoxel::PackedLabel>(m_maxPredictionVoxelCount);
  m_predictionVoxelLocationsMB = mbf.make_block<Vector3s>(m_maxPredictionVoxelCount);
  m_trainingFeaturesMB = mbf.make_block<float>(maxTrainingVoxelCount * featureCount);
  m_trainingLabelMaskMB = mbf.make_block<bool>(maxLabelCount);
  m_trainingVoxelCountsMB = mbf.make_block<unsigned int>(maxLabelCount);
  m_trainingVoxelLocationsMB = mbf.make_block<Vector3s>(maxTrainingVoxelCount);

  // Register the relevant decision function generators with the factory.
  DecisionFunctionGeneratorFactory<SpaintVoxel::Label>::instance().register_maker(
    SpaintDecisionFunctionGenerator::get_static_type(),
    &SpaintDecisionFunctionGenerator::maker
  );

  // Set up the random forest.
  reset_forest();
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void SemanticSegmentationComponent::reset_forest()
{
  const size_t treeCount = 5;
  DecisionTree<SpaintVoxel::Label>::Settings dtSettings(m_context->get_resources_dir() + "/RaflSettings.xml");
  m_forest.reset(new RandomForest<SpaintVoxel::Label>(treeCount, dtSettings));
}

void SemanticSegmentationComponent::run_feature_inspection(const VoxelRenderState_CPtr& renderState)
{
  // Get the voxels (if any) selected by the user (prior to selection transformation).
  Selector::Selection_CPtr selection = m_context->get_selector()->get_selection();

  // If the user hasn't selected a single voxel, early out.
  if(!selection || selection->dataSize != 1) return;

  // Calculate the feature descriptor for the selected voxel.
  boost::shared_ptr<ORUtils::MemoryBlock<float> > featuresMB = MemoryBlockFactory::instance().make_block<float>(m_featureCalculator->get_feature_count());
  m_featureCalculator->calculate_features(*selection, m_context->get_slam_state(m_sceneID)->get_voxel_scene().get(), *featuresMB);

#ifdef WITH_OPENCV
  // Convert the feature descriptor into an OpenCV image and show it in a window.
  featuresMB->UpdateHostFromDevice();
  const float *features = featuresMB->GetData(MEMORYDEVICE_CPU);
  const int patchSize = static_cast<int>(m_patchSize);
  cv::Mat3b featureInspectionImage = OpenCVUtil::make_rgb_image(features, patchSize, patchSize);

  const float scaleFactor = 10.0f;
  cv::resize(featureInspectionImage, featureInspectionImage, cv::Size(), scaleFactor, scaleFactor, CV_INTER_NN);

  cv::imshow("Feature Inspection", featureInspectionImage);
  const int delayMs = 1;
  cv::waitKey(delayMs);  // this is required in order to make OpenCV actually show the window
#endif
}

void SemanticSegmentationComponent::run_prediction(const VoxelRenderState_CPtr& renderState)
{
  // If we haven't been provided with a camera position from which to sample, early out.
  if(!renderState) return;

  // If the random forest is not yet valid, early out.
  if(!m_forest->is_valid()) return;

  // Sample some voxels for which to predict labels.
  m_predictionSampler->sample_voxels(renderState->raycastResult, m_maxPredictionVoxelCount, *m_predictionVoxelLocationsMB);

  // Calculate feature descriptors for the sampled voxels.
  m_featureCalculator->calculate_features(*m_predictionVoxelLocationsMB, m_context->get_slam_state(m_sceneID)->get_voxel_scene().get(), *m_predictionFeaturesMB);
  std::vector<Descriptor_CPtr> descriptors = ForestUtil::make_descriptors(*m_predictionFeaturesMB, m_maxPredictionVoxelCount, m_featureCalculator->get_feature_count());

  // Predict labels for the voxels based on the feature descriptors.
  SpaintVoxel::PackedLabel *labels = m_predictionLabelsMB->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < static_cast<int>(m_maxPredictionVoxelCount); ++i)
  {
    labels[i] = SpaintVoxel::PackedLabel(m_forest->predict(descriptors[i]), SpaintVoxel::LG_FOREST);
  }

  m_predictionLabelsMB->UpdateDeviceFromHost();

  // Mark the voxels with their predicted labels.
  m_context->mark_voxels(m_sceneID, m_predictionVoxelLocationsMB, m_predictionLabelsMB, NORMAL_MARKING);
}

void SemanticSegmentationComponent::run_training(const VoxelRenderState_CPtr& renderState)
{
  // If we haven't been provided with a camera position from which to sample, early out.
  if(!renderState) return;

  // Calculate a mask indicating the labels that are currently in use and from which we want to train.
  // Note that we deliberately avoid training from the background label (0), since the entire scene is
  // initially labelled as background and so training from the background would cause us to learn
  // incorrect labels for non-background things.
  LabelManager_CPtr labelManager = m_context->get_label_manager();
  const size_t maxLabelCount = labelManager->get_max_label_count();
  bool *labelMask = m_trainingLabelMaskMB->GetData(MEMORYDEVICE_CPU);
  labelMask[0] = false;
  for(size_t i = 1; i < maxLabelCount; ++i)
  {
    labelMask[i] = labelManager->has_label(static_cast<SpaintVoxel::Label>(i));
  }
  m_trainingLabelMaskMB->UpdateDeviceFromHost();

  // Sample voxels from the scene to use for training the random forest.
  const ORUtils::Image<Vector4f> *raycastResult = renderState->raycastResult;
  m_trainingSampler->sample_voxels(raycastResult, m_context->get_slam_state(m_sceneID)->get_voxel_scene().get(), *m_trainingLabelMaskMB, *m_trainingVoxelLocationsMB, *m_trainingVoxelCountsMB);

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
  m_featureCalculator->calculate_features(*m_trainingVoxelLocationsMB, m_context->get_slam_state(m_sceneID)->get_voxel_scene().get(), *m_trainingFeaturesMB);

  // Make the training examples.
  typedef boost::shared_ptr<const Example<SpaintVoxel::Label> > Example_CPtr;
  std::vector<Example_CPtr> examples = ForestUtil::make_examples<SpaintVoxel::Label>(
    *m_trainingFeaturesMB,
    *m_trainingVoxelCountsMB,
    m_featureCalculator->get_feature_count(),
    m_maxTrainingVoxelsPerLabel,
    maxLabelCount
  );

  // Train the forest.
  const size_t splitBudget = 20;
  m_forest->add_examples(examples);
  m_forest->train(splitBudget);
}

}
