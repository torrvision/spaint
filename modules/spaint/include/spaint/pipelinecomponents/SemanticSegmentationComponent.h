/**
 * spaint: SemanticSegmentationComponent.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SEMANTICSEGMENTATIONCOMPONENT
#define H_SPAINT_SEMANTICSEGMENTATIONCOMPONENT

#include <ITMLib/Objects/RenderStates/ITMRenderState.h>

#include <rafl/core/RandomForest.h>

#include "SemanticSegmentationContext.h"
#include "../features/interface/FeatureCalculator.h"
#include "../sampling/interface/PerLabelVoxelSampler.h"
#include "../sampling/interface/UniformVoxelSampler.h"
#include "../selectors/Selector.h"

namespace spaint {

/**
 * \brief An instance of this pipeline component can be used to semantically segment a scene.
 */
class SemanticSegmentationComponent
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<rafl::RandomForest<SpaintVoxel::Label> > RandomForest_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The shared context needed for semantic segmentation. */
  SemanticSegmentationContext_Ptr m_context;

  /** The feature calculator. */
  FeatureCalculator_CPtr m_featureCalculator;

  /** The random forest. */
  RandomForest_Ptr m_forest;

  /** The maximum number of voxels for which to predict labels each frame. */
  size_t m_maxPredictionVoxelCount;

  /** The maximum number of voxels per label from which to train each frame. */
  size_t m_maxTrainingVoxelsPerLabel;

  /** The side length of a VOP patch (must be odd). */
  size_t m_patchSize;

  /** A memory block in which to store the feature vectors computed for the various voxels during prediction. */
  boost::shared_ptr<ORUtils::MemoryBlock<float> > m_predictionFeaturesMB;

  /** A memory block in which to store the labels predicted for the various voxels. */
  boost::shared_ptr<ORUtils::MemoryBlock<SpaintVoxel::PackedLabel> > m_predictionLabelsMB;

  /** The voxel sampler used in prediction mode. */
  UniformVoxelSampler_CPtr m_predictionSampler;

  /** A memory block in which to store the locations of the voxels sampled for prediction purposes. */
  Selector::Selection_Ptr m_predictionVoxelLocationsMB;

  /** The ID of the scene on which the component should operate. */
  std::string m_sceneID;

  /** A memory block in which to store the feature vectors computed for the various voxels during training. */
  boost::shared_ptr<ORUtils::MemoryBlock<float> > m_trainingFeaturesMB;

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
   * \brief Constructs a semantic segmentation component.
   *
   * \param context The shared context needed for semantic segmentation.
   * \param sceneID The ID of the scene on which the component should operate.
   * \param seed    A seed for the random number generators used by the voxel samplers.
   */
  SemanticSegmentationComponent(const SemanticSegmentationContext_Ptr& context, const std::string& sceneID, unsigned int seed);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Resets the random forest.
   */
  void reset_forest();

  /**
   * \brief Runs the feature inspection section of the component.
   *
   * \param renderState The voxel render state associated with the camera position from which the user is picking voxels.
   */
  void run_feature_inspection(const VoxelRenderState_CPtr& renderState);

  /**
   * \brief Runs the prediction section of the component.
   *
   * \param renderState The voxel render state associated with the camera position from which to sample voxels.
   */
  void run_prediction(const VoxelRenderState_CPtr& renderState);

  /**
   * \brief Runs the training section of the component.
   *
   * \param renderState The voxel render state associated with the camera position from which to sample voxels.
   */
  void run_training(const VoxelRenderState_CPtr& renderState);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<SemanticSegmentationComponent> SemanticSegmentationComponent_Ptr;

}

#endif
