/**
 * spaint: TouchDetector.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_TOUCHDETECTOR
#define H_SPAINT_TOUCHDETECTOR

#include <arrayfire.h>

#include <itmx/base/ITMObjectPtrTypes.h>

#include <rafl/core/RandomForest.h>

#include <rigging/SimpleCamera.h>

#include <tvgutil/persistence/PropertyUtil.h>

#include "TouchSettings.h"
#include "../imageprocessing/interface/ImageProcessor.h"
#include "../visualisation/interface/DepthVisualiser.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to detect a touch interaction.
 */
class TouchDetector
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<af::array> AFArray_Ptr;
  typedef int Label;
  typedef rafl::RandomForest<Label> RF;
  typedef boost::shared_ptr<RF> RF_Ptr;

  //#################### PRIVATE DEBUGGING VARIABLES ####################
private:
  /** The number of milliseconds by which to delay between consecutive frames when debugging (0 = pause). */
  int m_debugDelayMs;

  /** The name of the debugging output window. */
  std::string m_touchDebuggingOutputWindowName;

  //#################### PRIVATE VARIABLES ####################
private:
  /** An image in which to store a mask of the changes that have been detected in the scene with respect to the reconstructed model. */
  AFArray_Ptr m_changeMask;

  /** An image in which to store the connected components of the change mask. */
  af::array m_connectedComponentImage;

  /** An image in which to store the depth of the reconstructed model as viewed from the current camera pose. */
  ITMFloatImage_Ptr m_depthRaycast;

  /** The depth visualiser. */
  DepthVisualiser_CPtr m_depthVisualiser;

  /** An image in which each pixel is the absolute difference (in m) between the raw depth image and the depth raycast. */
  AFArray_Ptr m_diffRawRaycast;

  /** The random forest used to score the candidate connected components. */
  RF_Ptr m_forest;

  /** The height of the images on which the touch detector is running. */
  int m_imageHeight;

  /** The image processor. */
  ImageProcessor_CPtr m_imageProcessor;

  /** The width of the images on which the touch detector is running. */
  int m_imageWidth;

  /** The settings to use for InfiniTAM. */
  Settings_CPtr m_itmSettings;

  /** The maximum area (in pixels) that a connected change component can have if it is to be considered as a candidate touch interaction. */
  int m_maxCandidateArea;

  /** The minimum area (in pixels) that a connected change component can have if it is to be considered as a candidate touch interaction. */
  int m_minCandidateArea;

  /** A thresholded version of the raw depth image captured from the camera in which parts of the scene > 2m away have been masked out. */
  ITMFloatImage_Ptr m_thresholdedRawDepth;

  /** An image in which to store a mask denoting the detected touch region. */
  AFArray_Ptr m_touchMask;

  /** The settings needed to configure the touch detector. */
  TouchSettings_Ptr m_touchSettings;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a touch detector.
   *
   * \param imgSize        The size of the images on which the touch detector is to run.
   * \param itmSettings    The settings to use for InfiniTAM.
   * \param touchSettings  The settings needed to configure the touch detector.
   */
  TouchDetector(const Vector2i& imgSize, const Settings_CPtr& itmSettings, const TouchSettings_Ptr& touchSettings);

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
  std::vector<Eigen::Vector2i> determine_touch_points(const rigging::MoveableCamera_CPtr& camera, const ITMFloatImage_CPtr& rawDepth, const VoxelRenderState_CPtr& renderState);

  /**
   * \brief Generates a colour image containing the current touch interaction (if any).
   *
   * \param view  The current view.
   * \return      A colour image containing the current touch interaction (if any).
   */
  ITMUChar4Image_CPtr generate_touch_image(const View_CPtr& view) const;

  /**
   * \brief Gets the depth of the reconstructed model as viewed from the current camera pose.
   *
   * \return  The depth of the reconstructed model as viewed from the current camera pose.
   */
  ITMFloatImage_CPtr get_depth_raycast() const;

  /**
   * \brief Gets an image in which each pixel is the absolute difference (in m) between the raw depth image and the depth raycast.
   *
   * \return  An image in which each pixel is the absolute difference (in m) between the raw depth image and the depth raycast.
   */
  ITMFloatImage_CPtr get_diff_raw_raycast() const;

  /**
   * \brief Gets a mask denoting the detected touch region.
   *
   * \return  A mask denoting the detected touch region.
   */
  ITMUCharImage_CPtr get_touch_mask() const;

  /**
   * \brief Gets a thresholded version of the raw depth image captured from the camera in which parts of the scene > 2m away have been masked out.
   *
   * \return  A thresholded version of the raw depth image captured from the camera in which parts of the scene > 2m away have been masked out.
   */
  ITMFloatImage_CPtr get_thresholded_raw_depth() const;

  /**
   * \brief Gets the depth value to use for pixels whose rays do not hit the scene when raycasting.
   *
   * \return  The depth value to use for pixels whose rays do not hit the scene when raycasting.
   */
  float invalid_depth_value() const;

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
   * Picks the candidate component most likely to correspond to a touch interaction based on mean distance to the scene.
   *
   * \param candidateComponents The IDs of components in the connected component image that denote candidate touch interactions.
   * \param diffRawRaycastInMm  An image in which each pixel is the absolute difference in mm between the current and raycasted depths.
   * \return                    The ID of the best candidate component.
   */
  int pick_best_candidate_component_based_on_distance(const af::array& candidateComponents, const af::array& diffRawRaycastInMm) const;

  /**
   * \brief Picks the candidate component most likely to correspond to a touch interaction based on predictions made by a random forest.
   *
   * If no candidates are classified as interactions by the forest, there is no best candidate and we return -1.
   *
   * \param candidateComponents The IDs of components in the connected component image that denote candidate touch interactions.
   * \param diffRawRaycastInMm  An image in which each pixel is the absolute difference in mm between the current and raycasted depths.
   * \return                    The ID of the best candidate component, or -1 if no candidates are classified as interactions by the forest.
   */
  int pick_best_candidate_component_based_on_forest(const af::array& candidateComponents, const af::array& diffRawRaycastInMm) const;

  /**
   * \brief Prepares a thresholded version of the raw depth image and a depth raycast ready for change detection.
   *
   * \param camera        The camera from which the scene is being rendered.
   * \param rawDepth      The raw depth image from the camera.
   * \param renderState   The render state corresponding to the camera.
   */
  void prepare_inputs(const rigging::MoveableCamera_CPtr& camera, const ITMFloatImage_CPtr& rawDepth, const VoxelRenderState_CPtr& renderState);

#ifdef WITH_OPENCV
  /**
   * \brief Sets up some debugging windows containing trackbars that can be used to control the values of
   *        various member variables, and updates those variables based on the current trackbar positions.
   */
  void process_debug_windows();

  /**
   * \brief Saves an image of each candidate component to disk for use with the touchtrain application.
   *
   * \param candidateComponents The candidate components denoting candidate touch interactions.
   * \param diffRawRaycast      An image in which each pixel is the absolute difference between the raw depth image and the depth raycast.
   */
  void save_candidate_components(const af::array& candidateComponents, const af::array& diffRawRaycastInMm) const;
#endif

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

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<TouchDetector> TouchDetector_Ptr;
typedef boost::shared_ptr<const TouchDetector> TouchDetector_CPtr;

}

#endif
