/**
 * spaint: TouchDetector.h
 */

#ifndef H_SPAINT_TOUCHDETECTOR
#define H_SPAINT_TOUCHDETECTOR

#include <arrayfire.h>

#include <ITMLib/Objects/ITMRenderState.h>
#include <ITMLib/Utils/ITMLibSettings.h>

#include <rigging/SimpleCamera.h>

#include "../imageprocessing/interface/ImageProcessor.h"
#include "../visualisers/interface/DepthVisualiser.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to detect a touch interaction.
 */
class TouchDetector
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<af::array> AFArray_Ptr;
  typedef boost::shared_ptr<ITMFloatImage> ITMFloatImage_Ptr;
  typedef boost::shared_ptr<const ITMFloatImage> ITMFloatImage_CPtr;
  typedef boost::shared_ptr<ITMUCharImage> ITMUCharImage_Ptr;
  typedef boost::shared_ptr<const ITMUCharImage> ITMUCharImage_CPtr;
  typedef boost::shared_ptr<const ITMLib::Objects::ITMRenderState> RenderState_CPtr;
  typedef boost::shared_ptr<const ITMLibSettings> Settings_CPtr;

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

  /** The height of the images on which the touch detector is running. */
  int m_imageHeight;

  /** The image processor. */
  boost::shared_ptr<const ImageProcessor> m_imageProcessor;

  /** The width of the images on which the touch detector is running. */
  int m_imageWidth;

  /** The threshold (in mm) below which the raw and raycasted depths are assumed to be equal. */
  int m_lowerDepthThresholdMm;

  /** The maximum area (in pixels) that a connected change component can have if it is to be considered as a candidate touch interaction. */
  int m_maxCandidateArea;

  /** The minimum area (in pixels) that a connected change component can have if it is to be considered as a candidate touch interaction. */
  int m_minCandidateArea;

  /** The side length of the morphological opening kernel that is applied to the change mask to reduce noise. */
  int m_morphKernelSize;

  /** The settings to use for InfiniTAM. */
  Settings_CPtr m_settings;

  /** A thresholded version of the raw depth image captured from the camera in which parts of the scene > 2m away have been masked out. */
  ITMFloatImage_Ptr m_thresholdedRawDepth;

  /** An Arrayfire image in which to store a mask denoting the detected touch region. */
  AFArray_Ptr m_touchMaskAF;

  /** An InfiniTAM image in which to store a mask denoting the detected touch region. */
  ITMUCharImage_Ptr m_touchMaskITM;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a touch detector.
   *
   * \param imgSize   The size of the images on which the touch detector is to run.
   * \param settings  The settings to use for InfiniTAM.
   */
  TouchDetector(const Vector2i& imgSize, const Settings_CPtr& settings);

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
   * \brief Gets the binary mask denoting the image regions in which a touch interaction is taking place.
   *
   * \return The touch mask.
   */
  ITMUCharImage_CPtr get_touch_mask() const;

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
  int pick_best_candidate_component(const af::array& candidateComponents, const af::array& diffRawRaycastInMm);

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
   * \brief Converts an Eigen Vector to an InfiniTAM vector.
   *
   * \param v  The Eigen vector.
   * \return   The InfiniTAM vector.
   */
  static Vector3f to_itm(const Eigen::Vector3f& v);
};

}

#endif
