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
 * \brief An instance of this class represents a processing pipeline for multiple scenes.
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

  //#################### NESTED TYPES ####################
private:
  /**
   * \brief An instance of this struct holds the pipeline components for an individual scene.
   */
  struct SingleScenePipeline
  {
    /** The object segmentation component for the scene. */
    spaint::ObjectSegmentationComponent_Ptr m_objectSegmentationComponent;

    /** The propagation component for the scene. */
    spaint::PropagationComponent_Ptr m_propagationComponent;

    /** The semantic segmentation component for the scene. */
    spaint::SemanticSegmentationComponent_Ptr m_semanticSegmentationComponent;

    /** The SLAM component for the scene. */
    spaint::SLAMComponent_Ptr m_slamComponent;

    /** The smoothing component for the scene. */
    spaint::SmoothingComponent_Ptr m_smoothingComponent;
  };

  //#################### PRIVATE VARIABLES ####################
private:
  /** The mode in which the multi-scene pipeline is currently running. */
  Mode m_mode;

  /** The spaint model. */
  Model_Ptr m_model;

  /** The pipelines for the individual scenes. */
  std::map<std::string,SingleScenePipeline> m_singleScenePipelines;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an instance of the multi-scene pipeline.
   *
   * \param settings      The settings to use for InfiniTAM.
   * \param resourcesDir  The path to the resources directory.
   * \param maxLabelCount The maximum number of labels that can be in use.
   */
  MultiScenePipeline(const Settings_Ptr& settings, const std::string& resourcesDir, size_t maxLabelCount);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Adds a pipeline for an individual scene to the multi-scene pipeline.
   *
   * \param sceneID           The ID of the individual scene.
   * \param imageSourceEngine The engine used to provide input images to the SLAM component for the scene.
   * \param seed              The seed to use for the random number generators used by the voxel samplers.
   * \param trackerType       The type of tracker to use when reconstructing the scene.
   * \param trackerParams     The parameters for the tracker (if any).
   * \param mappingMode       The mapping mode that the scene's SLAM component should use.
   * \param trackingMode      The tracking mode that the scene's SLAM component should use.
   */
  void add_single_scene_pipeline(const std::string& sceneID, const CompositeImageSourceEngine_Ptr& imageSourceEngine, unsigned int seed,
                                 spaint::TrackerType trackerType = spaint::TRACKER_INFINITAM, const std::string& trackerParams = "",
                                 spaint::SLAMComponent::MappingMode mappingMode = spaint::SLAMComponent::MAP_VOXELS_ONLY,
                                 spaint::SLAMComponent::TrackingMode trackingMode = spaint::SLAMComponent::TRACK_VOXELS);

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
   * \brief Resets the random forest for the specified scene.
   *
   * \param sceneID The scene ID.
   */
  void reset_forest(const std::string& sceneID);

  /**
   * \brief Runs the main section of the multi-scene pipeline.
   *
   * This involves processing the next frame (if any) for each individual scene.
   *
   * \return  true, if a new frame was available for every individual scene, or false otherwise.
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
   * \brief Sets the mode in which the multi-scene pipeline should now run.
   *
   * \param mode  The mode in which the multi-scene pipeline should now run.
   */
  void set_mode(Mode mode);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<MultiScenePipeline> MultiScenePipeline_Ptr;
typedef boost::shared_ptr<const MultiScenePipeline> MultiScenePipeline_CPtr;

#endif
