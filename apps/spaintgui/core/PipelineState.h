/**
 * spaintgui: PipelineState.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_PIPELINESTATE
#define H_SPAINTGUI_PIPELINESTATE

#include <spaint/pipelinecomponents/PropagationModel.h>
#include <spaint/pipelinecomponents/SemanticSegmentationModel.h>
#include <spaint/pipelinecomponents/SLAMModel.h>
#include <spaint/pipelinecomponents/SmoothingModel.h>

#include "Interactor.h"

/**
 * \brief An instance of this class represents the state shared between the different sections of the spaintgui processing pipeline.
 */
class PipelineState
: public spaint::PropagationModel,
  public spaint::SemanticSegmentationModel,
  public spaint::SLAMModel,
  public spaint::SmoothingModel
{
  //#################### TYPEDEFS ####################
private:
  typedef ITMLib::ITMScene<spaint::SpaintVoxel,ITMVoxelIndex> Scene;
  typedef boost::shared_ptr<Scene> Scene_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMView> View_Ptr;
  typedef boost::shared_ptr<const ITMLib::ITMVisualisationEngine<spaint::SpaintVoxel,ITMVoxelIndex> > VisualisationEngine_CPtr;

  //#################### PUBLIC VARIABLES ####################
public:
  /** The interactor that is used to interact with the InfiniTAM scene. */
  Interactor_Ptr m_interactor;

  /** The spaint model. */
  Model_Ptr m_model;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual const spaint::LabelManager_Ptr& get_label_manager() const
  {
    return m_model->get_label_manager();
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
