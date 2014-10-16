/**
 * spaint: SpaintEngine.h
 */

#ifndef H_SPAINT_SPAINTENGINE
#define H_SPAINT_SPAINTENGINE

#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

#include <Engine/ImageSourceEngine.h>

#include "../util/SpaintVoxel.h"

namespace spaint {

/**
 * \brief The main engine for spaint.
 */
class SpaintEngine
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<InfiniTAM::Engine::ImageSourceEngine> ImageSourceEngine_Ptr;
  typedef ITMScene<SpaintVoxel,ITMVoxelIndex> Scene;
  typedef boost::shared_ptr<Scene> Scene_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The engine used to provide input images to the fusion pipeline. */
  ImageSourceEngine_Ptr m_imageSourceEngine;

  /** The current model of the 3D scene. */
  Scene_Ptr m_scene;

  /** TODO */
  ITMLibSettings m_settings;

  //#################### CONSTRUCTORS ####################
public:
#if WITH_OPENNI
  /**
   * \brief Constructs an instance of the spaint engine that uses an OpenNI device as its image source.
   *
   * \param calibrationFilename The name of a file containing InfiniTAM calibration settings.
   * \param openNIDeviceURI     An optional OpenNI device URI (if boost::none is passed in, the default OpenNI device will be used).
   * \param settings            TODO
   */
  SpaintEngine(const std::string& calibrationFilename, const boost::optional<std::string>& openNIDeviceURI, const ITMLibSettings& settings);
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
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<SpaintEngine> SpaintEngine_Ptr;

}

#endif
