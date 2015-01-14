/**
 * spaint: SpaintPipeline.h
 */

#ifndef H_SPAINT_SPAINTPIPELINE
#define H_SPAINT_SPAINTPIPELINE

#include "SpaintModel.h"
#include "SpaintRaycaster.h"

namespace spaint {

/**
 * \brief An instance of this class is used to represent the spaint processing pipeline.
 */
class SpaintPipeline
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<InfiniTAM::Engine::ImageSourceEngine> ImageSourceEngine_Ptr;
  typedef boost::shared_ptr<ITMLowLevelEngine> LowLevelEngine_Ptr;
  typedef boost::shared_ptr<ITMSceneReconstructionEngine<SpaintVoxel,ITMVoxelIndex> > SceneReconstructionEngine_Ptr;
  typedef boost::shared_ptr<ITMSwappingEngine<SpaintVoxel,ITMVoxelIndex> > SwappingEngine_Ptr;
  typedef boost::shared_ptr<ITMTracker> Tracker_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The engine used to provide input images to the fusion pipeline. */
  ImageSourceEngine_Ptr m_imageSourceEngine;

  /** The engine used to perform low-level image processing operations. */
  LowLevelEngine_Ptr m_lowLevelEngine;

  /** The spaint model. */
  SpaintModel_Ptr m_model;

  /** A raycaster that can be used to cast rays into the InfiniTAM scene. */
  SpaintRaycaster_Ptr m_raycaster;

  /** Whether or not reconstruction has started yet (the tracking can only be run once it has). */
  bool m_reconstructionStarted;

  /** The engine used to perform fusion. */
  SceneReconstructionEngine_Ptr m_sceneReconstructionEngine;

  /** The settings to use for InfiniTAM. */
  ITMLibSettings m_settings;

  /** The engine controlling the swapping of voxel blocks in/out of GPU memory. */
  SwappingEngine_Ptr m_swappingEngine;

  /** The primary camera tracker. */
  Tracker_Ptr m_trackerPrimary;

  /** The secondary camera tracker. */
  Tracker_Ptr m_trackerSecondary;

  //#################### CONSTRUCTORS ####################
public:
#if WITH_OPENNI
  /**
   * \brief Constructs an instance of the spaint pipeline that uses an OpenNI device as its image source.
   *
   * \param calibrationFilename The name of a file containing InfiniTAM calibration settings.
   * \param openNIDeviceURI     An optional OpenNI device URI (if NULL is passed in, the default OpenNI device will be used).
   * \param settings            The settings to use for InfiniTAM.
   */
  SpaintPipeline(const std::string& calibrationFilename, const boost::shared_ptr<std::string>& openNIDeviceURI, const ITMLibSettings& settings);
#endif

  /**
   * \brief Constructs an instance of the spaint pipeline that uses images on disk as its image source.
   *
   * \param calibrationFilename The name of a file containing InfiniTAM calibration settings.
   * \param rgbImageMask        TODO
   * \param depthImageMask      TODO
   * \param settings            The settings to use for InfiniTAM.
   */
  SpaintPipeline(const std::string& calibrationFilename, const std::string& rgbImageMask, const std::string& depthImageMask, const ITMLibSettings& settings);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Processes the next frame from the image source engine.
   */
  void process_frame();

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Initialises the pipeline.
   */
  void initialise();
};

}

#endif
