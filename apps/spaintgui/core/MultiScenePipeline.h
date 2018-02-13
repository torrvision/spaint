/**
 * spaintgui: MultiScenePipeline.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_MULTISCENEPIPELINE
#define H_SPAINTGUI_MULTISCENEPIPELINE

#include <spaint/pipelinecomponents/ObjectSegmentationComponent.h>
#include <spaint/pipelinecomponents/PropagationComponent.h>
#include <spaint/pipelinecomponents/SemanticSegmentationComponent.h>
#include <spaint/pipelinecomponents/SLAMComponent.h>
#include <spaint/pipelinecomponents/SmoothingComponent.h>

#include "Model.h"

/**
 * \brief An instance of a class deriving from this one represents a processing pipeline for multiple scenes.
 */
class MultiScenePipeline
{
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

    /** In segmentation mode, the segmenter is used to separate its target from the rest of the scene. */
    MODE_SEGMENTATION,

    /** In segmentation training mode, the segmenter is trained to separate its target from the rest of the scene. */
    MODE_SEGMENTATION_TRAINING,

    /** In smoothing mode, voxel labels are filled in based on the labels of neighbouring voxels. */
    MODE_SMOOTHING,

    /** In train-and-predict mode, we alternate training and prediction to achieve a pleasing interactive effect. */
    MODE_TRAIN_AND_PREDICT,

    /** In training mode, a random forest is trained using voxels sampled from the current raycast. */
    MODE_TRAINING
  };

  //#################### PROTECTED VARIABLES ####################
protected:
  /** The mode in which the multi-scene pipeline is currently running. */
  Mode m_mode;

  /** The spaint model. */
  Model_Ptr m_model;

  /** The object segmentation components for the scenes. */
  std::map<std::string,spaint::ObjectSegmentationComponent_Ptr> m_objectSegmentationComponents;

  /** The propagation components for the scenes. */
  std::map<std::string,spaint::PropagationComponent_Ptr> m_propagationComponents;

  /** The semantic segmentation components for the scenes. */
  std::map<std::string,spaint::SemanticSegmentationComponent_Ptr> m_semanticSegmentationComponents;

  /** The SLAM components for the scenes. */
  std::map<std::string,spaint::SLAMComponent_Ptr> m_slamComponents;

  /** The smoothing components for the scenes. */
  std::map<std::string,spaint::SmoothingComponent_Ptr> m_smoothingComponents;

  /** The pipeline type. */
  std::string m_type;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a multi-scene pipeline.
   *
   * \param type          The pipeline type.
   * \param settings      The settings to use for InfiniTAM.
   * \param resourcesDir  The path to the resources directory.
   * \param maxLabelCount The maximum number of labels that can be in use.
   * \param mappingServer The remote mapping server (if any).
   */
  MultiScenePipeline(const std::string& type, const Settings_Ptr& settings, const std::string& resourcesDir, size_t maxLabelCount, const itmx::MappingServer_Ptr& mappingServer = itmx::MappingServer_Ptr());

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the multi-scene pipeline.
   */
  virtual ~MultiScenePipeline();

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Sets the mode in which the multi-scene pipeline should now run.
   *
   * \param mode  The mode in which the multi-scene pipeline should now run.
   */
  virtual void set_mode(Mode mode) = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets whether or not the user wants fusion to be run as part of the pipeline for the specified scene.
   *
   * \param sceneID The scene ID.
   * \return        true, if the user wants fusion to be run as part of the pipeline for the specified scene, or false otherwise.
   */
  bool get_fusion_enabled(const std::string& sceneID) const;

  /**
   * \brief Gets the mode in which the multi-scene pipeline is currently running.
   *
   * \return  The mode in which the multi-scene pipeline is currently running.
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
   * \brief Gets the pipeline type.
   *
   * \return  The pipeline type.
   */
  const std::string& get_type() const;

  /**
   * \brief Resets the random forest for the specified scene.
   *
   * \param sceneID The scene ID.
   */
  void reset_forest(const std::string& sceneID);

  /**
   * \brief Resets the specified scene.
   *
   * \param sceneID The scene ID.
   */
  void reset_scene(const std::string& sceneID);

  /**
   * \brief Runs the main section of the multi-scene pipeline.
   *
   * This involves processing the next frame (if any) for each individual scene.
   *
   * \return  true, if a new frame was available for the world scene, or false otherwise.
   */
  bool run_main_section();

  /**
   * \brief Runs the mode-specific section of the pipeline for the specified scene.
   *
   * \param sceneID     The scene ID.
   * \param renderState The render state to be used by the mode-specific section of the pipeline.
   */
  void run_mode_specific_section(const std::string& sceneID, const VoxelRenderState_CPtr& renderState);

  /**
   * \brief Sets whether or not the user wants fiducials to be detected in the specified scene.
   *
   * \param sceneID         The scene ID.
   * \param detectFiducials Whether or not the user wants fiducials to be detected in the specified scene.
   */
  void set_detect_fiducials(const std::string& sceneID, bool detectFiducials);

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
   * \brief Sets the mapping client (if any) for the specified scene.
   *
   * \param sceneID       The scene ID.
   * \param mappingClient The mapping client (if any) for the specified scene.
   */
  void set_mapping_client(const std::string& sceneID, const itmx::MappingClient_Ptr& mappingClient);

  /**
   * \brief Toggles whether or not the world scene's object segmentation component (if any) should write to its output pipe.
   */
  void toggle_segmentation_output();

  /**
   * \brief Alerts relevant components to the fact that the raycast result size has changed (e.g. because supersampling has been toggled).
   *
   * \param raycastResultSize The new raycast result size.
   */
  void update_raycast_result_size(int raycastResultSize);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<MultiScenePipeline> MultiScenePipeline_Ptr;
typedef boost::shared_ptr<const MultiScenePipeline> MultiScenePipeline_CPtr;

#endif
