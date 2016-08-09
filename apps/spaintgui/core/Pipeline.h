/**
 * spaintgui: Pipeline.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINTGUI_PIPELINE
#define H_SPAINTGUI_PIPELINE

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
  /** The state shared between the different sections of the pipeline. */
  PipelineState m_state;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an instance of the pipeline.
   *
   * \param imageSourceEngine The engine used to provide input images to the fusion pipeline.
   * \param settings          The settings to use for InfiniTAM.
   * \param resourcesDir      The path to the resouces directory.
   * \param trackerType       The type of tracker to use.
   * \param trackerParams     The parameters for the tracker (if any).
   */
  Pipeline(const CompositeImageSourceEngine_Ptr& imageSourceEngine, const Settings_Ptr& settings, const std::string& resourcesDir,
           TrackerType trackerType = TRACKER_INFINITAM, const std::string& trackerParams = "");

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

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Initialises the pipeline.
   *
   * \param settings  The settings to use for InfiniTAM.
   */
  void initialise(const Settings_Ptr& settings);

  /**
   * \brief Makes a hybrid tracker that refines the results of a primary tracker using ICP.
   *
   * \param primaryTracker    The primary tracker (e.g. a Rift or Vicon tracker).
   * \param settings          The settings to use for InfiniTAM.
   * \param scene             The scene.
   * \param rgbImageSize      The RGB image size.
   * \param depthImageSize    The depth image size.
   * \return                  The hybrid tracker.
   */
  ITMLib::ITMTracker *make_hybrid_tracker(ITMLib::ITMTracker *primaryTracker, const Settings_Ptr& settings, const Model::Scene_Ptr& scene,
                                          const Vector2i& rgbImageSize, const Vector2i& depthImageSize) const;

  /**
   * \brief Runs the section of the pipeline associated with feature inspection mode.
   *
   * \param renderState The render state associated with the camera position from which the user is picking voxels.
   */
  void run_feature_inspection_section(const RenderState_CPtr& renderState);

  /**
   * \brief Runs the section of the pipeline associated with prediction mode.
   *
   * \param samplingRenderState The render state associated with the camera position from which to sample voxels.
   */
  void run_prediction_section(const RenderState_CPtr& samplingRenderState);

  /**
   * \brief Runs the section of the pipeline associated with propagation mode.
   *
   * \param renderState The render state associated with the camera position from which to propagate.
   */
  void run_propagation_section(const RenderState_CPtr& renderState);

  /**
   * \brief Runs the section of the pipeline associated with smoothing mode.
   *
   * \param renderState The render state associated with the camera position from which to smoothe.
   */
  void run_smoothing_section(const RenderState_CPtr& renderState);

  /**
   * \brief Runs the section of the pipeline associated with training mode.
   *
   * \param samplingRenderState The render state associated with the camera position from which to sample voxels.
   */
  void run_training_section(const RenderState_CPtr& samplingRenderState);

  /**
   * \brief Sets up the tracker.
   *
   * \param settings          The settings to use for InfiniTAM.
   * \param scene             The scene.
   * \param rgbImageSize      The RGB image size.
   * \param depthImageSize    The depth image size.
   */
  void setup_tracker(const Settings_Ptr& settings, const Model::Scene_Ptr& scene, const Vector2i& rgbImageSize, const Vector2i& depthImageSize);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<Pipeline> Pipeline_Ptr;
typedef boost::shared_ptr<const Pipeline> Pipeline_CPtr;

#endif
