/**
 * spaintgui: PredictionSection.cpp
 */

#include "PredictionSection.h"

#include <rafl/base/Descriptor.h>
using namespace rafl;

#include <spaint/randomforest/ForestUtil.h>
using namespace spaint;

//#################### PUBLIC MEMBER FUNCTIONS ####################

void PredictionSection::run(PredictionState& state, const RenderState_CPtr& samplingRenderState)
{
  // If we haven't been provided with a camera position from which to sample, early out.
  if(!samplingRenderState) return;

  // If the random forest is not yet valid, early out.
  if(!state.get_forest()->is_valid()) return;

  // Sample some voxels for which to predict labels.
  const size_t maxPredictionVoxelCount = state.get_max_prediction_voxel_count();
  state.get_prediction_sampler()->sample_voxels(samplingRenderState->raycastResult, maxPredictionVoxelCount, *state.get_prediction_voxel_locations());

  // Calculate feature descriptors for the sampled voxels.
  state.get_feature_calculator()->calculate_features(*state.get_prediction_voxel_locations(), state.get_model()->get_scene().get(), *state.get_prediction_features());
  std::vector<Descriptor_CPtr> descriptors = ForestUtil::make_descriptors(*state.get_prediction_features(), maxPredictionVoxelCount, state.get_feature_calculator()->get_feature_count());

  // Predict labels for the voxels based on the feature descriptors.
  SpaintVoxel::PackedLabel *labels = state.get_prediction_labels()->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < static_cast<int>(maxPredictionVoxelCount); ++i)
  {
    labels[i] = SpaintVoxel::PackedLabel(state.get_forest()->predict(descriptors[i]), SpaintVoxel::LG_FOREST);
  }

  state.get_prediction_labels()->UpdateDeviceFromHost();

  // Mark the voxels with their predicted labels.
  state.get_interactor()->mark_voxels(state.get_prediction_voxel_locations(), state.get_prediction_labels());
}
