/**
 * spaintgui: TrainingSection.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "TrainingSection.h"

#include <rafl/examples/Example.h>
using namespace rafl;

#include <spaint/randomforest/ForestUtil.h>
#include <spaint/util/LabelManager.h>
using namespace spaint;

#define DEBUGGING 1

//#################### PUBLIC MEMBER FUNCTIONS ####################

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
  bool *labelMask = state.get_training_label_mask()->GetData(MEMORYDEVICE_CPU);
  labelMask[0] = false;
  for(size_t i = 1; i < maxLabelCount; ++i)
  {
    labelMask[i] = labelManager->has_label(static_cast<SpaintVoxel::Label>(i));
  }
  state.get_training_label_mask()->UpdateDeviceFromHost();

  // Sample voxels from the scene to use for training the random forest.
  const ORUtils::Image<Vector4f> *raycastResult = samplingRenderState->raycastResult;
  state.get_training_sampler()->sample_voxels(raycastResult, state.get_model()->get_scene().get(), *state.get_training_label_mask(), *state.get_training_voxel_locations(), *state.get_training_voxel_counts());

#if DEBUGGING
  // Output the numbers of voxels sampled for each label (for debugging purposes).
  for(size_t i = 0; i < state.get_training_voxel_counts()->dataSize; ++i)
  {
    std::cout << state.get_training_voxel_counts()->GetData(MEMORYDEVICE_CPU)[i] << ' ';
  }
  std::cout << '\n';

  // Make sure that the sampled voxels are available on the CPU so that they can be checked.
  state.get_training_voxel_locations()->UpdateHostFromDevice();
#endif

  // Compute feature vectors for the sampled voxels.
  state.get_feature_calculator()->calculate_features(*state.get_training_voxel_locations(), state.get_model()->get_scene().get(), *state.get_training_features());

  // Make the training examples.
  typedef boost::shared_ptr<const Example<SpaintVoxel::Label> > Example_CPtr;
  std::vector<Example_CPtr> examples = ForestUtil::make_examples<SpaintVoxel::Label>(
    *state.get_training_features(),
    *state.get_training_voxel_counts(),
    state.get_feature_calculator()->get_feature_count(),
    state.get_max_training_voxels_per_label(),
    maxLabelCount
  );

  // Train the forest.
  const size_t splitBudget = 20;
  state.get_forest()->add_examples(examples);
  state.get_forest()->train(splitBudget);
}
