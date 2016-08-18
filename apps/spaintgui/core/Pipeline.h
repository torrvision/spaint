/**
 * spaintgui: Pipeline.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINTGUI_PIPELINE
#define H_SPAINTGUI_PIPELINE

#include <spaint/pipelinecomponents/FeatureInspectionComponent.h>
#include <spaint/pipelinecomponents/PredictionComponent.h>
#include <spaint/pipelinecomponents/PropagationComponent.h>
#include <spaint/pipelinecomponents/SLAMComponent.h>
#include <spaint/pipelinecomponents/SmoothingComponent.h>
#include <spaint/pipelinecomponents/TrainingComponent.h>
#include <spaint/util/LabelManager.h>

#include "PipelineMode.h"
#include "PipelineState.h"

/**
 * \brief An instance of this class is used to represent the spaintgui processing pipeline.
 */
class Pipeline
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<InputSource::CompositeImageSourceEngine> CompositeImageSourceEngine_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMRenderState> RenderState_Ptr;
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;
  typedef boost::shared_ptr<ITMLib::ITMLibSettings> Settings_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMTrackingState> TrackingState_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMVisualisationEngine<spaint::SpaintVoxel,ITMVoxelIndex> > VisualisationEngine_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** TODO */
  spaint::FeatureInspectionComponent m_featureInspectionComponent;

  /** The mode in which the pipeline is currently running. */
  PipelineMode m_mode;

  /** TODO */
  spaint::PredictionComponent m_predictionComponent;

  /** TODO */
  spaint::PropagationComponent m_propagationComponent;

  /** The path to the resources directory. */
  std::string m_resourcesDir;

  /** TODO */
  spaint::SLAMComponent m_slamComponent;

  /** TODO */
  spaint::SmoothingComponent m_smoothingComponent;

  /** The state shared between the different sections of the pipeline. */
  PipelineState m_state;

  /** TODO */
  spaint::TrainingComponent m_trainingComponent;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an instance of the pipeline.
   *
   * \param imageSourceEngine The engine used to provide input images to the fusion pipeline.
   * \param settings          The settings to use for InfiniTAM.
   * \param resourcesDir      The path to the resouces directory.
   * \param labelManager      The label manager.
   * \param seed              The seed to use for the random number generators used by the voxel samplers.
   * \param trackerType       The type of tracker to use.
   * \param trackerParams     The parameters for the tracker (if any).
   */
  Pipeline(const CompositeImageSourceEngine_Ptr& imageSourceEngine, const Settings_Ptr& settings, const std::string& resourcesDir,
           const spaint::LabelManager_Ptr& labelManager, unsigned int seed, spaint::TrackerType trackerType = spaint::TRACKER_INFINITAM,
           const std::string& trackerParams = "");

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets whether or not the user wants fusion to be run as part of the pipeline.
   *
   * \return  true, if the user wants fusion to be run as part of the pipeline, or false otherwise.
   */
  bool get_fusion_enabled() const;

  /**
   * \brief Gets a copy of the image into which depth input is read each frame.
   *
   * \return  A copy of the image into which depth input is read each frame.
   */
  ITMShortImage_Ptr get_input_raw_depth_image_copy() const;

  /**
   * \brief Gets a copy of the image into which RGB input is read each frame.
   *
   * \return  A copy of the image into which RGB input is read each frame.
   */
  ITMUChar4Image_Ptr get_input_rgb_image_copy() const;

  /**
   * \brief Gets the interactor that is used to interact with the InfiniTAM scene.
   *
   * \return  The interactor that is used to interact with the InfiniTAM scene.
   */
  const Interactor_Ptr& get_interactor();

  /**
   * \brief Gets the render state corresponding to the live camera pose.
   *
   * \return  The render state corresponding to the live camera pose.
   */
  RenderState_CPtr get_live_render_state() const;

  /**
   * \brief Gets the mode in which the pipeline is currently running.
   *
   * \return  The mode in which the pipeline is currently running.
   */
  PipelineMode get_mode() const;

  /**
   * \brief Gets the spaint model.
   *
   * \return  The spaint model.
   */
  const Model_Ptr& get_model();

  /**
   * \brief Gets the spaint model.
   *
   * \return  The spaint model.
   */
  Model_CPtr get_model() const;

  /**
   * \brief Gets the raycaster that is used to cast rays into the InfiniTAM scene.
   *
   * \return  The raycaster that is used to cast rays into the InfiniTAM scene.
   */
  const Raycaster_Ptr& get_raycaster();

  /**
   * \brief Gets the raycaster that is used to cast rays into the InfiniTAM scene.
   *
   * \return  The raycaster that is used to cast rays into the InfiniTAM scene.
   */
  Raycaster_CPtr get_raycaster() const;

  /**
   * \brief Resets the random forest.
   */
  void reset_forest();

  /**
   * \brief Runs the main section of the pipeline.
   *
   * This involves processing the next frame from the image source engine (if any).
   *
   * \return  true, if a frame was available from the image source engine, or false otherwise.
   */
  bool run_main_section();

  /**
   * \brief Runs the mode-specific section of the pipeline.
   *
   * \param renderState The render state to be used by the mode-specific section of the pipeline.
   */
  void run_mode_specific_section(const RenderState_CPtr& renderState);

  /**
   * \brief Sets whether or not the user wants fusion to be run as part of the pipeline.
   *
   * Note: Just because the user wants fusion to be run doesn't mean that it necessarily will be on every frame.
   *       In particular, we prevent fusion when we know we have lost tracking, regardless of this setting.
   *
   * \param fusionEnabled Whether or not the user wants fusion to be run as part of the pipeline.
   */
  void set_fusion_enabled(bool fusionEnabled);

  /**
   * \brief Sets the mode in which the pipeline should now run.
   *
   * \param mode  The mode in which the pipeline should now run.
   */
  void set_mode(PipelineMode mode);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<Pipeline> Pipeline_Ptr;
typedef boost::shared_ptr<const Pipeline> Pipeline_CPtr;

#endif
