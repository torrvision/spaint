/**
 * spaint: SemanticSegmentationComponent.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SEMANTICSEGMENTATIONCOMPONENT
#define H_SPAINT_SEMANTICSEGMENTATIONCOMPONENT

#include <ITMLib/Objects/RenderStates/ITMRenderState.h>

#include "SemanticSegmentationModel.h"
#include "../sampling/interface/PerLabelVoxelSampler.h"
#include "../sampling/interface/UniformVoxelSampler.h"
#include "../selectors/Selector.h"

namespace spaint {

/**
 * \brief TODO
 */
class SemanticSegmentationComponent
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;
  typedef boost::shared_ptr<const ITMLib::ITMLibSettings> Settings_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The maximum number of labels that can be in use. */
  size_t m_maxLabelCount;

  /** The maximum number of voxels for which to predict labels each frame. */
  size_t m_maxPredictionVoxelCount;

  /** The maximum number of voxels per label from which to train each frame. */
  size_t m_maxTrainingVoxelsPerLabel;

  /** A memory block in which to store the labels predicted for the various voxels. */
  boost::shared_ptr<ORUtils::MemoryBlock<SpaintVoxel::PackedLabel> > m_predictionLabelsMB;

  /** The voxel sampler used in prediction mode. */
  UniformVoxelSampler_CPtr m_predictionSampler;

  /** A memory block in which to store the locations of the voxels sampled for prediction purposes. */
  Selector::Selection_Ptr m_predictionVoxelLocationsMB;

  /** A memory block in which to store a mask indicating which labels are currently in use and from which we want to train. */
  boost::shared_ptr<ORUtils::MemoryBlock<bool> > m_trainingLabelMaskMB;

  /** The voxel sampler used in training mode. */
  PerLabelVoxelSampler_CPtr m_trainingSampler;

  /** A memory block in which to store the number of voxels sampled for each label for training purposes. */
  boost::shared_ptr<ORUtils::MemoryBlock<unsigned int> > m_trainingVoxelCountsMB;

  /** A memory block in which to store the locations of the voxels sampled for training purposes. */
  Selector::Selection_Ptr m_trainingVoxelLocationsMB;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief TODO
   */
  SemanticSegmentationComponent(const Vector2i& depthImageSize, unsigned int seed, const Settings_CPtr& settings, size_t maxLabelCount);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  size_t get_max_prediction_voxel_count() const;

  /**
   * \brief TODO
   */
  size_t get_max_training_voxel_count() const;

  /** TODO */
  void run_feature_inspection(SemanticSegmentationModel& model, const RenderState_CPtr& renderState);

  /** TODO */
  void run_prediction(SemanticSegmentationModel& model, const RenderState_CPtr& samplingRenderState);

  /** TODO */
  void run_training(SemanticSegmentationModel& model, const RenderState_CPtr& samplingRenderState);
};

}

#endif
