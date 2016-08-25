/**
 * spaintgui: Pipeline.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINTGUI_PIPELINE
#define H_SPAINTGUI_PIPELINE

#include <spaint/pipelinecomponents/PropagationComponent.h>
#include <spaint/pipelinecomponents/SemanticSegmentationComponent.h>
#include <spaint/pipelinecomponents/SLAMComponent.h>
#include <spaint/pipelinecomponents/SmoothingComponent.h>
#include <spaint/visualisation/VisualisationGenerator.h>
#include <spaint/util/LabelManager.h>

#include "Model.h"

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

  //#################### ENUMERATIONS ####################
public:
  /**
   * \brief The different modes in which the pipeline can be running.
   */
  enum Mode
  {
    /** In feature inspection mode, the user can move the mouse around and visualise the features at particular points in the scene. */
    MODE_FEATURE_INSPECTION,

    /** In normal mode, the user can reconstruct and manually label the scene. */
    MODE_NORMAL,

    /** In prediction mode, the random forest is used to predict labels for previously-unseen voxels. */
    MODE_PREDICTION,

    /** In propagation mode, labels supplied by the user are propagated across surfaces in the scene. */
    MODE_PROPAGATION,

    /** In smoothing mode, voxel labels are filled in based on the labels of neighbouring voxels. */
    MODE_SMOOTHING,

    /** In train-and-predict mode, we alternate training and prediction to achieve a pleasing interactive effect. */
    MODE_TRAIN_AND_PREDICT,

    /** In training mode, a random forest is trained using voxels sampled from the current raycast. */
    MODE_TRAINING
  };

  //#################### NESTED TYPES ####################
private:
  /**
   * \brief TODO
   */
  struct ScenePipeline
  {
    /** TODO */
    spaint::PropagationComponent_Ptr m_propagationComponent;

    /** TODO */
    spaint::SemanticSegmentationComponent_Ptr m_semanticSegmentationComponent;

    /** TODO */
    spaint::SLAMComponent_Ptr m_slamComponent;

    /** TODO */
    spaint::SmoothingComponent_Ptr m_smoothingComponent;
  };

  //#################### PRIVATE VARIABLES ####################
private:
  /** The mode in which the pipeline is currently running. */
  Mode m_mode;

  /** The spaint model. */
  Model_Ptr m_model;

  /** TODO */
  std::map<std::string,ScenePipeline> m_scenePipelines;

  /** The visualiation generator that is used to render the InfiniTAM scene. */
  spaint::VisualisationGenerator_Ptr m_visualisationGenerator;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an instance of the pipeline.
   *
   * \param settings          The settings to use for InfiniTAM.
   * \param resourcesDir      The path to the resouces directory.
   * \param labelManager      The label manager.
   */
  Pipeline(const Settings_Ptr& settings, const std::string& resourcesDir, const spaint::LabelManager_Ptr& labelManager);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   *
   * \param sceneID           The scene ID.
   * \param imageSourceEngine The engine used to provide input images to the fusion pipeline.
   * \param seed              The seed to use for the random number generators used by the voxel samplers.
   * \param trackerType       The type of tracker to use.
   * \param trackerParams     The parameters for the tracker (if any).
   * \param mappingMode       The mapping mode to use for SLAM.
   */
  void add_scene_pipeline(const std::string& sceneID, const CompositeImageSourceEngine_Ptr& imageSourceEngine, unsigned int seed,
                          spaint::TrackerType trackerType = spaint::TRACKER_INFINITAM, const std::string& trackerParams = "",
                          spaint::SLAMComponent::MappingMode mappingMode = spaint::SLAMComponent::MAP_VOXELS_ONLY);

  /**
   * \brief Gets whether or not the user wants fusion to be run as part of the pipeline for the specified scene.
   *
   * \param sceneID The scene ID.
   * \return        true, if the user wants fusion to be run as part of the pipeline for the specified scene, or false otherwise.
   */
  bool get_fusion_enabled(const std::string& sceneID) const;

  /**
   * \brief Gets a copy of the image into which depth input is read each frame for the specified scene.
   *
   * \param sceneID The scene ID.
   * \return        A copy of the image into which depth input is read each frame for the specified scene.
   */
  ITMShortImage_Ptr get_input_raw_depth_image_copy(const std::string& sceneID) const;

  /**
   * \brief Gets a copy of the image into which RGB input is read each frame for the specified scene.
   *
   * \param sceneID The scene ID.
   * \return        A copy of the image into which RGB input is read each frame for the specified scene.
   */
  ITMUChar4Image_Ptr get_input_rgb_image_copy(const std::string& sceneID) const;

  /**
   * \brief Gets the mode in which the pipeline is currently running.
   *
   * \return  The mode in which the pipeline is currently running.
   */
  Mode get_mode() const;

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
   * \brief Gets the visualisation generator that is used to render the InfiniTAM scene.
   *
   * \return  The visualisation generator that is used to render the InfiniTAM scene.
   */
  spaint::VisualisationGenerator_CPtr get_visualisation_generator() const;

  /**
   * \brief Resets the random forest for the specified scene.
   *
   * \param sceneID The scene ID.
   */
  void reset_forest(const std::string& sceneID);

  /**
   * \brief Runs the main section of the pipeline.
   *
   * This involves processing the next frame from the image source engine (if any).
   *
   * \return  true, if a frame was available from the image source engine, or false otherwise.
   */
  bool run_main_section();

  /**
   * \brief Runs the mode-specific section of the pipeline for the specified scene.
   *
   * \param sceneID     The scene ID.
   * \param renderState The render state to be used by the mode-specific section of the pipeline.
   */
  void run_mode_specific_section(const std::string& sceneID, const RenderState_CPtr& renderState);

  /**
   * \brief Sets whether or not the user wants fusion to be run as part of the pipeline for the specified scene.
   *
   * Note: Just because the user wants fusion to be run doesn't mean that it necessarily will be on every frame.
   *       In particular, we prevent fusion when we know we have lost tracking, regardless of this setting.
   *
   * \param sceneID       The scene ID.
   * \param fusionEnabled Whether or not the user wants fusion to be run as part of the pipeline for the specified scene.
   */
  void set_fusion_enabled(const std::string& sceneID, bool fusionEnabled);

  /**
   * \brief Sets the mode in which the pipeline should now run.
   *
   * \param mode  The mode in which the pipeline should now run.
   */
  void set_mode(Mode mode);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<Pipeline> Pipeline_Ptr;
typedef boost::shared_ptr<const Pipeline> Pipeline_CPtr;

#endif
