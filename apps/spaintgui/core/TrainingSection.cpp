/**
 * spaintgui: TrainingSection.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "TrainingSection.h"

#include <rafl/examples/Example.h>
using namespace rafl;

#include <spaint/randomforest/ForestUtil.h>
#include <spaint/sampling/VoxelSamplerFactory.h>
#include <spaint/util/LabelManager.h>
#include <spaint/util/MemoryBlockFactory.h>
using namespace spaint;

#define DEBUGGING 1

//#################### CONSTRUCTORS ####################

TrainingSection::TrainingSection(const Vector2i& depthImageSize, unsigned int seed, const Settings_CPtr& settings, size_t maxLabelCount)
: m_maxLabelCount(maxLabelCount),
  m_maxTrainingVoxelsPerLabel(128) // FIXME: This value shouldn't be hard-coded here ultimately.
{
  // Set up the voxel sampler.
  const int raycastResultSize = depthImageSize.width * depthImageSize.height;
  m_trainingSampler = VoxelSamplerFactory::make_per_label_sampler(maxLabelCount, m_maxTrainingVoxelsPerLabel, raycastResultSize, seed, settings->deviceType);

  // Set up the memory blocks.
  MemoryBlockFactory& mbf = MemoryBlockFactory::instance();
  m_trainingLabelMaskMB = mbf.make_block<bool>(maxLabelCount);
  m_trainingVoxelCountsMB = mbf.make_block<unsigned int>(maxLabelCount);
  m_trainingVoxelLocationsMB = mbf.make_block<Vector3s>(get_max_training_voxel_count());
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

size_t TrainingSection::get_max_training_voxel_count() const
{
  return m_maxLabelCount * m_maxTrainingVoxelsPerLabel;
}

void TrainingSection::run(TrainingState& state, const RenderState_CPtr& samplingRenderState)
{
  // If we haven't been provided with a camera position from which to sample, early out.
  if(!samplingRenderState) return;

  // Calculate a mask indicating the labels that are currently in use and from which we want to train.
  // Note that we deliberately avoid training from the background label (0), since the entire scene is
  // initially labelled as background and so training from the background would cause us to learn
  // incorrect labels for non-background things.
  LabelManager_CPtr labelManager = state.get_model()->get_label_manager();
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
  m_trainingSampler->sample_voxels(raycastResult, state.get_model()->get_scene().get(), *m_trainingLabelMaskMB, *m_trainingVoxelLocationsMB, *m_trainingVoxelCountsMB);

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
  state.get_feature_calculator()->calculate_features(*m_trainingVoxelLocationsMB, state.get_model()->get_scene().get(), *state.get_training_features());

  // Make the training examples.
  typedef boost::shared_ptr<const Example<SpaintVoxel::Label> > Example_CPtr;
  std::vector<Example_CPtr> examples = ForestUtil::make_examples<SpaintVoxel::Label>(
    *state.get_training_features(),
    *m_trainingVoxelCountsMB,
    state.get_feature_calculator()->get_feature_count(),
    m_maxTrainingVoxelsPerLabel,
    maxLabelCount
  );

  // Train the forest.
  const size_t splitBudget = 20;
  state.get_forest()->add_examples(examples);
  state.get_forest()->train(splitBudget);
}
