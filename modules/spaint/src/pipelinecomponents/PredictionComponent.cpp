/**
 * spaint: PredictionComponent.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/PredictionComponent.h"

#include <rafl/base/Descriptor.h>
using namespace rafl;

#include "randomforest/ForestUtil.h"
#include "sampling/VoxelSamplerFactory.h"
#include "util/MemoryBlockFactory.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

PredictionComponent::PredictionComponent(const Vector2i& depthImageSize, unsigned int seed, const Settings_CPtr& settings)
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

size_t PredictionComponent::get_max_prediction_voxel_count() const
{
  return m_maxPredictionVoxelCount;
}

void PredictionComponent::run(PredictionModel& model, const RenderState_CPtr& samplingRenderState)
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

}
