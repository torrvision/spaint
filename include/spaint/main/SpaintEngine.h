/**
 * spaint: SpaintEngine.h
 */

#ifndef H_SPAINT_SPAINTENGINE
#define H_SPAINT_SPAINTENGINE

#include <Engine/ImageSourceEngine.h>

#include "../util/SharedPtr.h"
#include "../util/SpaintVoxel.h"

namespace spaint {

/**
 * \brief The main engine for spaint.
 */
class SpaintEngine
{
  //#################### TYPEDEFS ####################
private:
  typedef spaint::shared_ptr<InfiniTAM::Engine::ImageSourceEngine> ImageSourceEngine_Ptr;
  typedef spaint::shared_ptr<ITMLowLevelEngine> LowLevelEngine_Ptr;
  typedef ITMScene<SpaintVoxel,ITMVoxelIndex> Scene;
  typedef spaint::shared_ptr<Scene> Scene_Ptr;
  typedef spaint::shared_ptr<ITMSceneReconstructionEngine<SpaintVoxel,ITMVoxelIndex> > SceneReconstructionEngine_Ptr;
  typedef spaint::shared_ptr<ITMSwappingEngine<SpaintVoxel,ITMVoxelIndex> > SwappingEngine_Ptr;
  typedef spaint::shared_ptr<ITMTracker> Tracker_Ptr;
  typedef spaint::shared_ptr<ITMTrackingState> TrackingState_Ptr;
  typedef spaint::shared_ptr<ITMView> View_Ptr;
  typedef spaint::shared_ptr<ITMVisualisationEngine<SpaintVoxel,ITMVoxelIndex> > VisualisationEngine_Ptr;
  typedef spaint::shared_ptr<ITMVisualisationState> VisualisationState_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The engine used to provide input images to the fusion pipeline. */
  ImageSourceEngine_Ptr m_imageSourceEngine;

  /** The engine used to perform low-level image processing operations. */
  LowLevelEngine_Ptr m_lowLevelEngine;

  /** Whether or not reconstruction has started yet (the tracker can only be run once it has). */
  bool m_reconstructionStarted;

  /** The current model of the 3D scene. */
  Scene_Ptr m_scene;

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

  /** The current tracking state (containing the camera pose and additional tracking information used by InfiniTAM). */
  TrackingState_Ptr m_trackingState;

  /** The current view of the scene. */
  View_Ptr m_view;

  /** The engine used for raycasting the resulting model, etc. */
  VisualisationEngine_Ptr m_visualisationEngine;

  /** The current visualisation state. */
  VisualisationState_Ptr m_visualisationState;

  //#################### CONSTRUCTORS ####################
public:
#if WITH_OPENNI
  /**
   * \brief Constructs an instance of the spaint engine that uses an OpenNI device as its image source.
   *
   * \param calibrationFilename The name of a file containing InfiniTAM calibration settings.
   * \param openNIDeviceURI     An optional OpenNI device URI (if NULL is passed in, the default OpenNI device will be used).
   * \param settings            TODO
   */
  SpaintEngine(const std::string& calibrationFilename, const spaint::shared_ptr<std::string>& openNIDeviceURI, const ITMLibSettings& settings);
#endif

  /**
   * \brief Constructs an instance of the spaint engine that uses images on disk as its image source.
   *
   * \param calibrationFilename The name of a file containing InfiniTAM calibration settings.
   * \param rgbImageMask        TODO
   * \param depthImageMask      TODO
   * \param settings            TODO
   */
  SpaintEngine(const std::string& calibrationFilename, const std::string& rgbImageMask, const std::string& depthImageMask, const ITMLibSettings& settings);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Processes the next frame from the image source engine.
   */
  void process_frame();

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Initialises the engine.
   */
  void initialise();
};

//#################### TYPEDEFS ####################

typedef spaint::shared_ptr<SpaintEngine> SpaintEngine_Ptr;

}

#endif
