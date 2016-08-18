/**
 * spaintgui: PipelineState.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_PIPELINESTATE
#define H_SPAINTGUI_PIPELINESTATE

#include <rafl/core/RandomForest.h>

#include <spaint/features/interface/FeatureCalculator.h>
#include <spaint/pipelinecomponents/FeatureInspectionModel.h>
#include <spaint/pipelinecomponents/PredictionModel.h>
#include <spaint/pipelinecomponents/PropagationModel.h>
#include <spaint/pipelinecomponents/SLAMModel.h>
#include <spaint/pipelinecomponents/SmoothingModel.h>
#include <spaint/pipelinecomponents/TrainingModel.h>
#include <spaint/smoothing/interface/LabelSmoother.h>
#include <spaint/util/SpaintVoxel.h>

#include "Interactor.h"

/**
 * \brief An instance of this class represents the state shared between the different sections of the spaintgui processing pipeline.
 */
class PipelineState
: public spaint::FeatureInspectionModel,
  public spaint::PredictionModel,
  public spaint::PropagationModel,
  public spaint::SLAMModel,
  public spaint::SmoothingModel,
  public spaint::TrainingModel
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<rafl::RandomForest<spaint::SpaintVoxel::Label> > RandomForest_Ptr;
  typedef ITMLib::ITMScene<spaint::SpaintVoxel,ITMVoxelIndex> Scene;
  typedef boost::shared_ptr<Scene> Scene_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMView> View_Ptr;
  typedef boost::shared_ptr<const ITMLib::ITMVisualisationEngine<spaint::SpaintVoxel,ITMVoxelIndex> > VisualisationEngine_CPtr;

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
  virtual const spaint::LabelManager_Ptr& get_label_manager() const
  {
    return m_model->get_label_manager();
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
  virtual const Scene_Ptr& get_scene()
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
    return m_model->get_semantic_label();
  }

  /** Override */
  virtual const boost::shared_ptr<ORUtils::MemoryBlock<float> >& get_training_features() const
  {
    return m_trainingFeaturesMB;
  }

  /** Override */
  virtual const View_Ptr& get_view()
  {
    return m_model->get_view();
  }

  /** Override */
  virtual VisualisationEngine_CPtr get_visualisation_engine() const
  {
    return m_model->get_visualisation_engine();
  }

  /** Override */
  virtual const spaint::VoxelMarker_CPtr& get_voxel_marker() const
  {
    return m_interactor->get_voxel_marker();
  }

  /** Override */
  virtual void set_view(ITMLib::ITMView *view)
  {
    m_model->set_view(view);
  }
};

#endif
