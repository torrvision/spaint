/**
 * spaintgui: Pipeline.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINTGUI_PIPELINE
#define H_SPAINTGUI_PIPELINE

#include <boost/optional.hpp>

#include <InputSource/CompositeImageSourceEngine.h>
#include <ITMLib/Core/ITMDenseMapper.h>
#include <ITMLib/Core/ITMTrackingController.h>
#include <ITMLib/Engines/LowLevel/Interface/ITMLowLevelEngine.h>
#include <ITMLib/Engines/ViewBuilding/Interface/ITMViewBuilder.h>
#include <ITMLib/Objects/Misc/ITMIMUCalibrator.h>
#include <RelocLib/PoseDatabase.h>
#include <RelocLib/Relocaliser.h>

#include <rafl/core/RandomForest.h>

#include <spaint/features/interface/FeatureCalculator.h>
#include <spaint/propagation/interface/LabelPropagator.h>
#include <spaint/sampling/interface/PerLabelVoxelSampler.h>
#include <spaint/sampling/interface/UniformVoxelSampler.h>
#include <spaint/segmentation/Segmenter.h>
#include <spaint/smoothing/interface/LabelSmoother.h>
#include <spaint/trackers/FallibleTracker.h>

#include <tvgutil/filesystem/SequentialPathGenerator.h>

#include "Interactor.h"
#include "Model.h"
#include "Raycaster.h"

/**
 * \brief An instance of this class is used to represent the spaintgui processing pipeline.
 */
class Pipeline
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<InputSource::CompositeImageSourceEngine> CompositeImageSourceEngine_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMDenseMapper<spaint::SpaintVoxel,ITMVoxelIndex> > DenseMapper_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMIMUCalibrator> IMUCalibrator_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMLowLevelEngine> LowLevelEngine_Ptr;
  typedef boost::shared_ptr<RelocLib::PoseDatabase> PoseDatabase_Ptr;
  typedef boost::shared_ptr<rafl::RandomForest<spaint::SpaintVoxel::Label> > RandomForest_Ptr;
  typedef boost::shared_ptr<RelocLib::Relocaliser> Relocaliser_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMRenderState> RenderState_Ptr;
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;
  typedef boost::shared_ptr<ITMLib::ITMLibSettings> Settings_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMTracker> ITMTracker_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMTrackingController> TrackingController_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMTrackingState> TrackingState_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMViewBuilder> ViewBuilder_Ptr;
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

  /**
   * \brief The different tracker types we can use.
   */
  enum TrackerType
  {
    TRACKER_INFINITAM,
    TRACKER_RIFT,
    TRACKER_ROBUSTVICON,
    TRACKER_VICON
  };

  //#################### PRIVATE VARIABLES ####################
private:
  /** The dense mapper. */
  DenseMapper_Ptr m_denseMapper;

  /** A pointer to a tracker that can detect tracking failures (if available). */
  spaint::FallibleTracker *m_fallibleTracker;

  /** The feature calculator. */
  spaint::FeatureCalculator_CPtr m_featureCalculator;

  /** The name to give the featuere inspection window. */
  std::string m_featureInspectionWindowName;

  /** The random forest. */
  RandomForest_Ptr m_forest;

  /** The number of frames for which fusion has been run. */
  size_t m_fusedFramesCount;

  /** Whether or not the user wants fusion to be run as part of the pipeline. */
  bool m_fusionEnabled;

  /** The engine used to provide input images to the fusion pipeline. */
  CompositeImageSourceEngine_Ptr m_imageSourceEngine;

  /** The IMU calibrator. */
  IMUCalibrator_Ptr m_imuCalibrator;

  /**
   * A number of initial frames to fuse, regardless of their tracking quality.
   * Tracking quality can be poor in the first few frames, when there is only
   * a limited model against which to track. By forcibly fusing these frames,
   * we prevent poor tracking quality from stopping the reconstruction. After
   * these frames have been fused, only frames with a good tracking result will
   * be fused.
   */
  size_t m_initialFramesToFuse;

  /** The image into which depth input is read each frame. */
  ITMShortImage_Ptr m_inputRawDepthImage;

  /** The image into which RGB input is read each frame. */
  ITMUChar4Image_Ptr m_inputRGBImage;

  /** The interactor that is used to interact with the InfiniTAM scene. */
  Interactor_Ptr m_interactor;

  /** The remaining number of frames for which we need to achieve good tracking before we can add another keyframe. */
  size_t m_keyframeDelay;

  /** The label propagator. */
  spaint::LabelPropagator_CPtr m_labelPropagator;

  /** The label smoother. */
  spaint::LabelSmoother_CPtr m_labelSmoother;

  /** The engine used to perform low-level image processing operations. */
  LowLevelEngine_Ptr m_lowLevelEngine;

  /** The maximum number of voxels for which to predict labels each frame. */
  size_t m_maxPredictionVoxelCount;

  /** The maximum number of voxels per label from which to train each frame. */
  size_t m_maxTrainingVoxelsPerLabel;

  /** The mode in which the pipeline is currently running. */
  Mode m_mode;

  /** The spaint model. */
  Model_Ptr m_model;

  /** The side length of a VOP patch (must be odd). */
  size_t m_patchSize;

  /** The database of previous poses for relocalisation. */
  PoseDatabase_Ptr m_poseDatabase;

  /** A memory block in which to store the feature vectors computed for the various voxels during prediction. */
  boost::shared_ptr<ORUtils::MemoryBlock<float> > m_predictionFeaturesMB;

  /** A memory block in which to store the labels predicted for the various voxels. */
  boost::shared_ptr<ORUtils::MemoryBlock<spaint::SpaintVoxel::PackedLabel> > m_predictionLabelsMB;

  /** The voxel sampler used in prediction mode. */
  spaint::UniformVoxelSampler_CPtr m_predictionSampler;

  /** A memory block in which to store the locations of the voxels sampled for prediction purposes. */
  spaint::Selector::Selection_Ptr m_predictionVoxelLocationsMB;

  /** The raycaster that is used to cast rays into the InfiniTAM scene. */
  Raycaster_Ptr m_raycaster;

  /** The relocaliser. */
  Relocaliser_Ptr m_relocaliser;

  /** The path to the resources directory. */
  std::string m_resourcesDir;

  /** The path generator for the current segmentation video (if any). */
  boost::optional<tvgutil::SequentialPathGenerator> m_segmentationPathGenerator;

  /** The segmenter. */
  mutable spaint::Segmenter_Ptr m_segmenter;

  /** The tracker. */
  ITMTracker_Ptr m_tracker;

  /**
   * The parameters for the tracker (if any). For example, this would be the host on which the
   * Vicon software is running (e.g. "<IP address>:<port>") if we're using the Vicon tracker.
   */
  std::string m_trackerParams;

  /** The type of tracker to use. */
  TrackerType m_trackerType;

  /** The tracking controller. */
  TrackingController_Ptr m_trackingController;

  /** A memory block in which to store the feature vectors computed for the various voxels during training. */
  boost::shared_ptr<ORUtils::MemoryBlock<float> > m_trainingFeaturesMB;

  /** A memory block in which to store a mask indicating which labels are currently in use and from which we want to train. */
  boost::shared_ptr<ORUtils::MemoryBlock<bool> > m_trainingLabelMaskMB;

  /** The voxel sampler used in training mode. */
  spaint::PerLabelVoxelSampler_CPtr m_trainingSampler;

  /** A memory block in which to store the number of voxels sampled for each label for training purposes. */
  boost::shared_ptr<ORUtils::MemoryBlock<unsigned int> > m_trainingVoxelCountsMB;

  /** A memory block in which to store the locations of the voxels sampled for training purposes. */
  spaint::Selector::Selection_Ptr m_trainingVoxelLocationsMB;

  /** The view builder. */
  ViewBuilder_Ptr m_viewBuilder;

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
  void set_mode(Mode mode);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Gets the segmenter.
   *
   * \return  The segmenter.
   */
  const spaint::Segmenter_Ptr& get_segmenter() const;

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
   * \brief Runs the section of the pipeline associated with segmentation mode.
   *
   * \param renderState The render state associated with the camera position from which to segment the target.
   */
  void run_segmentation_section(const RenderState_CPtr& renderState);

  /**
   * \brief Runs the section of the pipeline associated with segmentation training mode.
   *
   * \param renderState The render state associated with the camera position from which to train the segmenter.
   */
  void run_segmentation_training_section(const RenderState_CPtr& renderState);

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
