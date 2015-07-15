/**
 * spaint: TouchDetector.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_TOUCHDETECTOR
#define H_SPAINT_TOUCHDETECTOR

#include <arrayfire.h>

#include <ITMLib/Objects/ITMRenderState.h>
#include <ITMLib/Utils/ITMLibSettings.h>

#include <rafl/core/RandomForest.h>

#include <rigging/SimpleCamera.h>

#include <tvgutil/PropertyUtil.h>

#include "../imageprocessing/interface/ImageProcessor.h"
#include "../visualisers/interface/DepthVisualiser.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to detect a touch interaction.
 */
class TouchDetector
{
  //#################### NESTED TYPES ####################
public:
  /**
   * \brief An instance of this calss can be used to provide the settings needed to configure a touch detector.
   */
  class Settings
  {
    //~~~~~~~~~~~~~~~~~~~~ TYPEDEFS ~~~~~~~~~~~~~~~~~~~~
  private:

    //~~~~~~~~~~~~~~~~~~~~ PUBLIC VARIABLES ~~~~~~~~~~~~~~~~~~~~
  public:
    /** The path to the random forest used to filter touch regions. */
    std::string forestPath;

    /** The threshold (in mm) below which the raw and raycasted depths are assumed to be equal. */
    int lowerDepthThresholdMm;

    /** TODO. */
    float minCandidateFraction;

    /** TODO. */
    float minTouchAreaFraction;

    /** TODO. */
    float maxCandidateFraction;

    /** The side length of the morphological opening kernel that is applied to the change mask to reduce noise. */
    int morphKernelSize;

    /** A flag indicating whether to save images of the candidate connected components. */
    bool saveCandidateComponents;

    /** The path to use when saving the candidate components. */
    std::string saveCandidateComponentsPath;

    //~~~~~~~~~~~~~~~~~~~~ CONSTRUCTORS ~~~~~~~~~~~~~~~~~~~~
  public:
    /**
     * \brief Default constructor.
     */
    Settings();

    /**
     * \brief Attempts to load settings fromt he specifiedXML file.
     *
     * This will throw if the settings cannot be successfully loaded.
     *
     * \param filename The name of the file.
     */
    explicit Settings(const std::string& filename);

    //~~~~~~~~~~~~~~~~~~~~ PRIVATE MEMBER FUNCTIONS ~~~~~~~~~~~~~~~~~~~~
  private:
    /**
     * \brief Load settings froma property map.
     *
     * \param properties  The property map.
     */
    void initialise(const std::map<std::string,std::string>& properties);
  };

  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<af::array> AFArray_Ptr;
  typedef boost::shared_ptr<ITMFloatImage> ITMFloatImage_Ptr;
  typedef boost::shared_ptr<const ITMFloatImage> ITMFloatImage_CPtr;
  typedef boost::shared_ptr<ITMUCharImage> ITMUCharImage_Ptr;
  typedef boost::shared_ptr<ITMUChar4Image> ITMUChar4Image_Ptr;
  typedef boost::shared_ptr<const ITMUChar4Image> ITMUChar4Image_CPtr;
  typedef boost::shared_ptr<const ITMLib::Objects::ITMRenderState> RenderState_CPtr;
  typedef boost::shared_ptr<const ITMLibSettings> ITMSettings_CPtr;
  typedef boost::shared_ptr<const ITMView> View_CPtr;
  typedef int Label;
  typedef rafl::RandomForest<Label> RF;
  typedef boost::shared_ptr<RF> RF_Ptr;

  //#################### PRIVATE DEBUGGING VARIABLES ####################
private:
  /** The number of milliseconds by which to delay between consecutive frames when debugging (0 = pause). */
  int m_debugDelayMs;

  /** The name of the debugging output window. */
  std::string m_debuggingOutputWindowName;

  /** The name of the morphological operator output window. */
  std::string m_morphologicalOperatorWindowName;

  //#################### PRIVATE VARIABLES ####################
private:
  /** An image in which to store a mask of the changes that have been detected in the scene with respect to the reconstructed model. */
  af::array m_changeMask;

  /** An image in which to store the connected components of the change mask. */
  af::array m_connectedComponentImage;

  /** An image in which to store the depth of the reconstructed model as viewed from the current camera pose. */
  ITMFloatImage_Ptr m_depthRaycast;

  /** The depth visualiser. */
  boost::shared_ptr<const DepthVisualiser> m_depthVisualiser;

  /** An image in which each pixel is the absolute difference between the raw depth image and the depth raycast. */
  AFArray_Ptr m_diffRawRaycast;

  /** A random forest used to score the candidate connected components. */
  RF_Ptr m_forest;

  /** The height of the images on which the touch detector is running. */
  int m_imageHeight;

  /** The image processor. */
  ImageProcessor_CPtr m_imageProcessor;

  /** The width of the images on which the touch detector is running. */
  int m_imageWidth;

  /** The maximum area (in pixels) that a connected change component can have if it is to be considered as a candidate touch interaction. */
  int m_maxCandidateArea;

  /** The minimum area (in pixels) that a connected change component can have if it is to be considered as a candidate touch interaction. */
  int m_minCandidateArea;

  /** The settings to use for InfiniTAM. */
  ITMSettings_CPtr m_itmSettings;

  /** A thresholded version of the raw depth image captured from the camera in which parts of the scene > 2m away have been masked out. */
  ITMFloatImage_Ptr m_thresholdedRawDepth;

  /** An image in which to store a mask denoting the detected touch region. */
  AFArray_Ptr m_touchMask;

  /** The settings needed to configure the touch detector. */
  Settings m_touchSettings;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a touch detector.
   *
   * \param imgSize        The size of the images on which the touch detector is to run.
   * \param itmSettings    The settings to use for InfiniTAM.
   * \param touchSettings  The settings needed to configure the touch detector.
   */
  TouchDetector(const Vector2i& imgSize, const ITMSettings_CPtr& itmSettings, const Settings& touchSettings);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Determines the points (if any) that the user is touching in the scene.
   *
   * \param camera        The camera from which the scene is being rendered.
   * \param rawDepth      The raw depth image from the camera.
   * \param renderState   The render state corresponding to the camera.
   * \return              The points (if any) that the user is touching in the scene.
   */
  std::vector<Eigen::Vector2i> determine_touch_points(const rigging::MoveableCamera_CPtr& camera, const ITMFloatImage_CPtr& rawDepth, const RenderState_CPtr& renderState);

  /**
   * \brief Generates a colour image containing the current touch interaction (if any).
   *
   * \param view  The current view.
   * \return      A colour image containing the current touch interaction (if any).
   */
  ITMUChar4Image_CPtr generate_touch_image(const View_CPtr& view) const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Detects changes between the raw depth image from the camera and a depth raycast of the reconstructed model.
   */
  void detect_changes();

  /**
   * \brief Extracts a set of touch points from the specified component in the connected component image.
   *
   * \param component           The ID of a component in the connected component image.
   * \param diffRawRaycastInMm  An image in which each pixel is the absolute difference in mm between the current and raycasted depths.
   * \return                    The touch points extracted from the specified component.
   */
  std::vector<Eigen::Vector2i> extract_touch_points(int component, const af::array& diffRawRaycastInMm);

  /**
   * Picks the candidate component most likely to correspond to a touch interaction.
   *
   * \param candidateComponents The IDs of components in the connected component image that denote candidate touch interactions.
   * \param diffRawRaycastInMm  An image in which each pixel is the absolute difference in mm between the current and raycasted depths.
   * \return                    The ID of the best candidate component.
   */
  int pick_best_candidate_component_based_on_distance(const af::array& candidateComponents, const af::array& diffRawRaycastInMm);

  /**
   * \brief Picks the candidate component most likely to correspond to a touch interaction based on predictions made by a random forest.
   *
   * \param candidateComponents The IDs of components in the connected omponent image that denote candidate touch interactions.
   * \param diffRawRaycastInMm  An image in which each pixel is the absolute difference in mm between the current and raycasted depths.
   * \return                    The IS of the best candidate component, or -1 if none are found.
   */
  int pick_best_candidate_component_based_on_forest(const af::array& candidateComponents, const af::array& diffRawRaycastInMm);

  /**
   * \brief Prepares a thresholded version of the raw depth image and a depth raycast ready for change detection.
   *
   * \param camera        The camera from which the scene is being rendered.
   * \param rawDepth      The raw depth image from the camera.
   * \param renderState   The render state corresponding to the camera.
   */
  void prepare_inputs(const rigging::MoveableCamera_CPtr& camera, const ITMFloatImage_CPtr& rawDepth, const RenderState_CPtr& renderState);

  /**
   * \brief Sets up some debugging windows containing trackbars that can be used to control the values of
   *        various member variables, and updates those variables based on the current trackbar positions.
   */
  void process_debug_windows();

  /**
   * \brief Select candidate connected components that fall within a certain size range.
   *
   * \return  The IDs of the candidate components.
   */
  af::array select_candidate_components();

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Makes a copy of an ArrayFire array in which the elements have been clamped to the specified range.
   *
   * \param arr   An ArrayFire array.
   * \param lower The lower bound of the range to which to clamp.
   * \param upper The upper bound of the range to which to clamp.
   * \return      A copy of the array in which the elements have been clamped to the specified range.
   */
  static af::array clamp_to_range(const af::array& arr, float lower, float upper);

  /**
   * \brief Gets the number of files contained in a directory.
   *
   * \param path  The path of the directory.
   * \return      The number of files in the specified directory.
   */
  static size_t get_file_count(const std::string& path);

  /**
   * \brief Saves the candidate component images to a directory.
   *
   * \param candidateComponents The IDs of components in the connected component image that denote candidate touch interactions.
   * \param diffRawRaycastInMm  An image in which each pixel is the absolute difference in mm between the current and raycasted depths.
   */
  void save_candidate_components(const af::array& candidateComponents, const af::array& diffRawRaycastInMm) const;

  /**
   * \brief Converts an Eigen Vector to an InfiniTAM vector.
   *
   * \param v  The Eigen vector.
   * \return   The InfiniTAM vector.
   */
  static Vector3f to_itm(const Eigen::Vector3f& v);
};

}

#endif
