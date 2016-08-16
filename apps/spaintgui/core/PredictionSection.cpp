/**
 * spaintgui: PredictionSection.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "PredictionSection.h"

#include <rafl/base/Descriptor.h>
using namespace rafl;

#include <spaint/randomforest/ForestUtil.h>
#include <spaint/sampling/VoxelSamplerFactory.h>
#include <spaint/util/MemoryBlockFactory.h>
using namespace spaint;

//#################### CONSTRUCTORS ####################

PredictionSection::PredictionSection(const Vector2i& depthImageSize, unsigned int seed, const Settings_CPtr& settings)
{
  // Set up the voxel sampler.
  const int raycastResultSize = depthImageSize.width * depthImageSize.height;
  m_predictionSampler = VoxelSamplerFactory::make_uniform_sampler(raycastResultSize, seed, settings->deviceType);

  // Set the maximum numbers of voxels to use for prediction.
  // FIXME: These values shouldn't be hard-coded here ultimately.
#ifndef USE_LOW_POWER_MODE
  m_maxPredictionVoxelCount = 8192;
#else
  m_maxPredictionVoxelCount = 512;
#endif

  // Set up the memory blocks.
  MemoryBlockFactory& mbf = MemoryBlockFactory::instance();
  m_predictionLabelsMB = mbf.make_block<SpaintVoxel::PackedLabel>(m_maxPredictionVoxelCount);
  m_predictionVoxelLocationsMB = mbf.make_block<Vector3s>(m_maxPredictionVoxelCount);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

size_t PredictionSection::get_max_prediction_voxel_count() const
{
  return m_maxPredictionVoxelCount;
}

void PredictionSection::run(PredictionState& state, const RenderState_CPtr& samplingRenderState)
{
  // If we haven't been provided with a camera position from which to sample, early out.
  if(!samplingRenderState) return;

  // If the random forest is not yet valid, early out.
  if(!state.get_forest()->is_valid()) return;

  // Sample some voxels for which to predict labels.
  m_predictionSampler->sample_voxels(samplingRenderState->raycastResult, m_maxPredictionVoxelCount, *m_predictionVoxelLocationsMB);

  // Calculate feature descriptors for the sampled voxels.
  state.get_feature_calculator()->calculate_features(*m_predictionVoxelLocationsMB, state.get_model()->get_scene().get(), *state.get_prediction_features());
  std::vector<Descriptor_CPtr> descriptors = ForestUtil::make_descriptors(*state.get_prediction_features(), m_maxPredictionVoxelCount, state.get_feature_calculator()->get_feature_count());

  // Predict labels for the voxels based on the feature descriptors.
  SpaintVoxel::PackedLabel *labels = m_predictionLabelsMB->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < static_cast<int>(m_maxPredictionVoxelCount); ++i)
  {
    labels[i] = SpaintVoxel::PackedLabel(state.get_forest()->predict(descriptors[i]), SpaintVoxel::LG_FOREST);
  }

  m_predictionLabelsMB->UpdateDeviceFromHost();

  // Mark the voxels with their predicted labels.
  state.get_interactor()->mark_voxels(m_predictionVoxelLocationsMB, m_predictionLabelsMB);
}
