/**
 * spaintgui: PipelineState.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_PIPELINESTATE
#define H_SPAINTGUI_PIPELINESTATE

#include <rafl/core/RandomForest.h>

#include <spaint/features/interface/FeatureCalculator.h>
#include <spaint/smoothing/interface/LabelSmoother.h>
#include <spaint/util/SpaintVoxel.h>

#include "FeatureInspectionState.h"
#include "Interactor.h"
#include "PredictionState.h"
#include "PropagationState.h"
#include "SLAMState.h"
#include "SmoothingState.h"
#include "TrainingState.h"

/**
 * \brief An instance of this class represents the state shared between the different sections of the spaintgui processing pipeline.
 */
class PipelineState
: public FeatureInspectionState,
  public PredictionState,
  public PropagationState,
  public SLAMState,
  public SmoothingState,
  public TrainingState
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<rafl::RandomForest<spaint::SpaintVoxel::Label> > RandomForest_Ptr;
  typedef ITMLib::ITMScene<spaint::SpaintVoxel,ITMVoxelIndex> Scene;
  typedef boost::shared_ptr<Scene> Scene_Ptr;

  //#################### PUBLIC VARIABLES ####################
public:
  /** The feature calculator. */
  spaint::FeatureCalculator_CPtr m_featureCalculator;

  /** The random forest. */
  RandomForest_Ptr m_forest;

  /** The interactor that is used to interact with the InfiniTAM scene. */
  Interactor_Ptr m_interactor;

  /** The spaint model. */
  Model_Ptr m_model;

  /** The side length of a VOP patch (must be odd). */
  size_t m_patchSize;

  /** A memory block in which to store the feature vectors computed for the various voxels during prediction. */
  boost::shared_ptr<ORUtils::MemoryBlock<float> > m_predictionFeaturesMB;

  /** The raycaster that is used to cast rays into the InfiniTAM scene. */
  Raycaster_Ptr m_raycaster;

  /** A memory block in which to store the feature vectors computed for the various voxels during training. */
  boost::shared_ptr<ORUtils::MemoryBlock<float> > m_trainingFeaturesMB;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual const spaint::FeatureCalculator_CPtr& get_feature_calculator() const
  {
    return m_featureCalculator;
  }

  /** Override */
  virtual const RandomForest_Ptr& get_forest()
  {
    return m_forest;
  }

  /** Override */
  virtual const Interactor_Ptr& get_interactor() const
  {
    return m_interactor;
  }

  /** Override */
  virtual const spaint::LabelManager_Ptr& get_label_manager() const
  {
    return m_model->get_label_manager();
  }

  /** Override */
  virtual const Model_Ptr& get_model() const
  {
    return m_model;
  }

  /** Override */
  virtual size_t get_patch_size() const
  {
    return m_patchSize;
  }

  /** Override */
  virtual const boost::shared_ptr<ORUtils::MemoryBlock<float> >& get_prediction_features()
  {
    return m_predictionFeaturesMB;
  }

  /** Override */
  virtual const Raycaster_Ptr& get_raycaster() const
  {
    return m_raycaster;
  }

  /** Override */
  virtual const Scene_Ptr& get_scene() const
  {
    return m_model->get_scene();
  }

  /** Override */
  virtual spaint::Selector_CPtr get_selector() const
  {
    return m_interactor->get_selector();
  }

  /** Override */
  virtual spaint::SpaintVoxel::Label get_semantic_label() const
  {
    return m_interactor->get_semantic_label();
  }

  /** Override */
  virtual const boost::shared_ptr<ORUtils::MemoryBlock<float> >& get_training_features() const
  {
    return m_trainingFeaturesMB;
  }

  /** Override */
  virtual const spaint::VoxelMarker_CPtr& get_voxel_marker() const
  {
    return m_interactor->get_voxel_marker();
  }
};

#endif
