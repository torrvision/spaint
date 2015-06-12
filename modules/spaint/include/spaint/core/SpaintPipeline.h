/**
 * spaint: SpaintPipeline.h
 */

#ifndef H_SPAINT_SPAINTPIPELINE
#define H_SPAINT_SPAINTPIPELINE

#include <boost/optional.hpp>

#include <rafl/RandomForest.h>

#include "SpaintInteractor.h"
#include "SpaintModel.h"
#include "SpaintRaycaster.h"

#include "../features/interface/FeatureCalculator.h"
#include "../sampling/interface/UniformVoxelSampler.h"
#include "../sampling/interface/VoxelSampler.h"

#ifdef WITH_VICON
#include "../trackers/ViconTracker.h"
#endif

namespace spaint {

/**
 * \brief An instance of this class is used to represent the spaint processing pipeline.
 */
class SpaintPipeline
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<ITMDenseMapper<SpaintVoxel,ITMVoxelIndex> > DenseMapper_Ptr;
  typedef boost::shared_ptr<InfiniTAM::Engine::ImageSourceEngine> ImageSourceEngine_Ptr;
  typedef boost::shared_ptr<ITMIMUCalibrator> IMUCalibrator_Ptr;
  typedef boost::shared_ptr<ITMShortImage> ITMShortImage_Ptr;
  typedef boost::shared_ptr<ITMUChar4Image> ITMUChar4Image_Ptr;
  typedef boost::shared_ptr<ITMLowLevelEngine> LowLevelEngine_Ptr;
  typedef boost::shared_ptr<rafl::RandomForest<SpaintVoxel::LabelType> > RandomForest_Ptr;
  typedef boost::shared_ptr<ITMRenderState> RenderState_Ptr;
  typedef boost::shared_ptr<const ITMRenderState> RenderState_CPtr;
  typedef boost::shared_ptr<ITMLibSettings> Settings_Ptr;
  typedef boost::shared_ptr<ITMTracker> ITMTracker_Ptr;
  typedef boost::shared_ptr<ITMTrackingController> TrackingController_Ptr;
  typedef boost::shared_ptr<ITMTrackingState> TrackingState_Ptr;
  typedef boost::shared_ptr<ITMViewBuilder> ViewBuilder_Ptr;
  typedef boost::shared_ptr<ITMVisualisationEngine<SpaintVoxel,ITMVoxelIndex> > VisualisationEngine_Ptr;

  //#################### ENUMERATIONS ####################
public:
  /**
   * \brief The different modes in which the pipeline can be running.
   */
  enum Mode
  {
    /** In normal mode, the user can reconstruct and manually label the scene. */
    MODE_NORMAL,

    /** In prediction mode, the random forest is used to predict labels for previously-unseen voxels. */
    MODE_PREDICTION,

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
    TRACKER_VICON
  };

  //#################### PRIVATE VARIABLES ####################
private:
  /** The dense mapper. */
  DenseMapper_Ptr m_denseMapper;

  /** The feature calculator. */
  FeatureCalculator_CPtr m_featureCalculator;

  /** A memory block in which to store the feature vectors computed for the various voxels during training. */
  boost::shared_ptr<ORUtils::MemoryBlock<float> > m_featuresMB;

  /** The random forest. */
  RandomForest_Ptr m_forest;

  /** Whether or not fusion should be run as part of the pipeline. */
  bool m_fusionEnabled;

  /** The engine used to provide input images to the fusion pipeline. */
  ImageSourceEngine_Ptr m_imageSourceEngine;

  /** The IMU calibrator. */
  IMUCalibrator_Ptr m_imuCalibrator;

  /** The image into which depth input is to be read each frame. */
  ITMShortImage_Ptr m_inputRawDepthImage;

  /** The image into which RGB input is to be read each frame. */
  ITMUChar4Image_Ptr m_inputRGBImage;

  /** The interactor that is used to interact with the InfiniTAM scene. */
  SpaintInteractor_Ptr m_interactor;

  /** A memory block in which to store a mask indicating which labels are currently in use. */
  boost::shared_ptr<ORUtils::MemoryBlock<bool> > m_labelMaskMB;

  /** The engine used to perform low-level image processing operations. */
  LowLevelEngine_Ptr m_lowLevelEngine;

  /** The mode in which the pipeline is currently running. */
  Mode m_mode;

  /** The spaint model. */
  SpaintModel_Ptr m_model;

  /** The voxel sampler used in prediction mode. */
  UniformVoxelSampler_CPtr m_predictionSampler;

  /** The raycaster that is used to cast rays into the InfiniTAM scene. */
  SpaintRaycaster_Ptr m_raycaster;

  /** Whether or not reconstruction has started yet (the tracking can only be run once it has). */
  bool m_reconstructionStarted;

  /** A memory block in which to store the number of voxels sampled for each label. */
  boost::shared_ptr<ORUtils::MemoryBlock<unsigned int> > m_sampledVoxelCountsMB;

  /** A memory block in which to store the locations of the voxels sampled for each label. */
  Selector::Selection_Ptr m_sampledVoxelLocationsMB;

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

#ifdef WITH_VICON
  /** The Vicon tracker (we keep a pointer to it so that we can check whether tracking has been lost). */
  ViconTracker *m_viconTracker;
#endif

  /** The view builder. */
  ViewBuilder_Ptr m_viewBuilder;

  /** The voxel sampler. */
  VoxelSampler_CPtr m_voxelSampler;

  //#################### CONSTRUCTORS ####################
public:
#ifdef WITH_OPENNI
  /**
   * \brief Constructs an instance of the spaint pipeline that uses an OpenNI device as its image source.
   *
   * \param calibrationFilename The name of a file containing InfiniTAM calibration settings.
   * \param openNIDeviceURI     An optional OpenNI device URI (if boost::none is passed in, the default OpenNI device will be used).
   * \param settings            The settings to use for InfiniTAM.
   * \param trackerType         The type of tracker to use.
   * \param trackerParams       The parameters for the tracker (if any).
   */
  SpaintPipeline(const std::string& calibrationFilename, const boost::optional<std::string>& openNIDeviceURI, const Settings_Ptr& settings,
                 TrackerType trackerType = TRACKER_INFINITAM, const std::string& trackerParams = "");
#endif

  /**
   * \brief Constructs an instance of the spaint pipeline that uses images on disk as its image source.
   *
   * \param calibrationFilename The name of a file containing InfiniTAM calibration settings.
   * \param rgbImageMask        The mask for the RGB image filenames (e.g. "Teddy/Frames/%04i.ppm").
   * \param depthImageMask      The mask for the depth image filenames (e.g. "Teddy/Frames/%04i.pgm").
   * \param settings            The settings to use for InfiniTAM.
   */
  SpaintPipeline(const std::string& calibrationFilename, const std::string& rgbImageMask, const std::string& depthImageMask, const Settings_Ptr& settings);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets whether or not fusion is currently being run as part of the pipeline.
   *
   * \return  true, if fusion is currently being run as part of the pipeline, or false otherwise.
   */
  bool get_fusion_enabled() const;

  /**
   * \brief Gets the interactor that is used to interact with the InfiniTAM scene.
   *
   * \return  The interactor that is used to interact with the InfiniTAM scene.
   */
  const SpaintInteractor_Ptr& get_interactor();

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
  const SpaintModel_Ptr& get_model();

  /**
   * \brief Gets the spaint model.
   *
   * \return  The spaint model.
   */
  SpaintModel_CPtr get_model() const;

  /**
   * \brief Gets the raycaster that is used to cast rays into the InfiniTAM scene.
   *
   * \return  The raycaster that is used to cast rays into the InfiniTAM scene.
   */
  const SpaintRaycaster_Ptr& get_raycaster();

  /**
   * \brief Gets the raycaster that is used to cast rays into the InfiniTAM scene.
   *
   * \return  The raycaster that is used to cast rays into the InfiniTAM scene.
   */
  SpaintRaycaster_CPtr get_raycaster() const;

  /**
   * \brief Runs the main section of the pipeline.
   *
   * This involves processing the next frame from the image source engine.
   */
  void run_main_section();

  /**
   * \brief Runs the mode-specific section of the pipeline.
   *
   * \param samplingRenderState The render state associated with the camera position from which to sample voxels.
   */
  void run_mode_specific_section(const RenderState_CPtr& samplingRenderState);

  /**
   * \brief Sets whether or not fusion should be run as part of the pipeline.
   *
   * \param fusionEnabled Whether or not fusion should be run as part of the pipeline.
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
   * \param trackedImageSize  The tracked image size.
   * \return                  The hybrid tracker.
   */
  ITMTracker *make_hybrid_tracker(ITMTracker *primaryTracker, const Settings_Ptr& settings, const SpaintModel::Scene_Ptr& scene, const Vector2i& trackedImageSize) const;

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
   * \param trackedImageSize  The tracked image size.
   */
  void setup_tracker(const Settings_Ptr& settings, const SpaintModel::Scene_Ptr& scene, const Vector2i& trackedImageSize);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<SpaintPipeline> SpaintPipeline_Ptr;
typedef boost::shared_ptr<const SpaintPipeline> SpaintPipeline_CPtr;

}

#endif
