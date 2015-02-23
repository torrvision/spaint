/**
 * spaint: SpaintPipeline.h
 */

#ifndef H_SPAINT_SPAINTPIPELINE
#define H_SPAINT_SPAINTPIPELINE

#include "SpaintModel.h"
#include "SpaintRaycaster.h"

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
  typedef boost::shared_ptr<ITMRenderState> RenderState_Ptr;
  typedef boost::shared_ptr<ITMLibSettings> Settings_Ptr;
  typedef boost::shared_ptr<ITMTracker> ITMTracker_Ptr;
  typedef boost::shared_ptr<ITMTrackingController> TrackingController_Ptr;
  typedef boost::shared_ptr<ITMTrackingState> TrackingState_Ptr;
  typedef boost::shared_ptr<ITMViewBuilder> ViewBuilder_Ptr;
  typedef boost::shared_ptr<ITMVisualisationEngine<SpaintVoxel,ITMVoxelIndex> > VisualisationEngine_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The dense mapper. */
  DenseMapper_Ptr m_denseMapper;

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

  /** The engine used to perform low-level image processing operations. */
  LowLevelEngine_Ptr m_lowLevelEngine;

  /** The spaint model. */
  SpaintModel_Ptr m_model;

  /** The raycaster that is used to cast rays into the InfiniTAM scene. */
  SpaintRaycaster_Ptr m_raycaster;

  /** Whether or not reconstruction has started yet (the tracking can only be run once it has). */
  bool m_reconstructionStarted;

  /** The tracker. */
  ITMTracker_Ptr m_tracker;

  /** The tracking controller. */
  TrackingController_Ptr m_trackingController;

#ifdef WITH_VICON
  /** The host on which the Vicon software is running (e.g. "<IP address>:<port>"), if we're using the Vicon tracker. */
  std::string m_viconHost;

  /** The Vicon tracker (we keep a pointer to it so that we can check whether tracking has been lost). */
  ViconTracker *m_viconTracker;
#endif

  /** The view builder. */
  ViewBuilder_Ptr m_viewBuilder;

  //#################### CONSTRUCTORS ####################
public:
#ifdef WITH_OPENNI
  /**
   * \brief Constructs an instance of the spaint pipeline that uses an OpenNI device as its image source.
   *
   * \param calibrationFilename The name of a file containing InfiniTAM calibration settings.
   * \param openNIDeviceURI     An optional OpenNI device URI (if boost::none is passed in, the default OpenNI device will be used).
   * \param settings            The settings to use for InfiniTAM.
   */
  SpaintPipeline(const std::string& calibrationFilename, const boost::optional<std::string>& openNIDeviceURI, const Settings_Ptr& settings);

#ifdef WITH_VICON
  /**
   * \brief Constructs an instance of the spaint pipeline that uses an OpenNI device as its image source and a Vicon system for tracking.
   *
   * \param calibrationFilename The name of a file containing InfiniTAM calibration settings.
   * \param openNIDeviceURI     An optional OpenNI device URI (if boost::none is passed in, the default OpenNI device will be used).
   * \param settings            The settings to use for InfiniTAM.
   * \param viconHost           The host on which the Vicon software is running (e.g. "<IP address>:<port>"), if we're using the Vicon tracker.
   */
  SpaintPipeline(const std::string& calibrationFilename, const boost::optional<std::string>& openNIDeviceURI, const Settings_Ptr& settings, const std::string& viconHost);
#endif
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
  SpaintRaycaster_CPtr get_raycaster() const;

  /**
   * \brief Processes the next frame from the image source engine.
   */
  void process_frame();

  /**
   * \brief Sets whether or not fusion should be run as part of the pipeline.
   *
   * \param fusionEnabled Whether or not fusion should be run as part of the pipeline.
   */
  void set_fusion_enabled(bool fusionEnabled);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Initialises the pipeline.
   *
   * \param settings  The settings to use for InfiniTAM.
   */
  void initialise(const Settings_Ptr& settings);

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
