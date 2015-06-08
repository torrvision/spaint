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
  typedef boost::shared_ptr<const ITMLib::Objects::ITMRenderState> RenderState_CPtr;
  typedef std::vector<Eigen::Vector2i> Points;
  typedef boost::shared_ptr<Points> Points_Ptr;
  typedef boost::shared_ptr<const Points> Points_CPtr;
  typedef boost::shared_ptr<const ITMLibSettings> Settings_CPtr;

  //#################### PRIVATE DEBUGGING VARIABLES ####################
private:
  /** The delay between processing frames (0 = pause). */
  int m_debugDelayms;

  /** The lower depth threshold value in millimeters needed to the opencv trackbar. */
  int m_depthLowerThresholdmm;

  //#################### PRIVATE VARIABLES ####################
private:
  /** An image in which to store a mask of the changes that have been detected in the scene with respect to the reconstructed model. */
  af::array m_changeMask;

  /** The number of columns in the image matrix, (width). */
  int m_cols;

  /** An image in which to store the connected components of the change mask. */
  af::array m_connectedComponentImage;

  /** The depth calculator. */
  boost::shared_ptr<const DepthVisualiser> m_depthCalculator;

  /** The threshold below which the raw and raycasted depth is assumed to be equal. */
  float m_depthLowerThreshold;

  /** An image into which to store the depth of the currently visible scene from the camera. */
  ITMFloatImage_Ptr m_depthRaycast;

  /** The threshold above which any difference in the raw and raycasted depth is ignored. */
  float m_depthUpperThreshold;

  /** An image in which each pixel is the absolute difference between the current and raycasted depths. */
  AFArray_Ptr m_diffRawRaycast;

  /** Multiplatform image processing tools. */
  boost::shared_ptr<const ImageProcessor> m_imageProcessor;

  /** The maximum image area a connected component region can take, expressed as a percentage of the number of pixels in the image. */
  float m_maximumConnectedComponentAreaPercentage;

  /** The maximum image area a connected component region can take, expressed in square-pixels. */
  int m_maximumConnectedComponentAreaThreshold;

  /** The minimum image area a connected component region can take, expressed as a percentage of the number of pixels in the image. */
  float m_minimumConnectedComponentAreaPercentage;

  /** The minimum image area a connected component region can take, expressed in square-pixels. */
  int m_minimumConnectedComponentAreaThreshold;

  /** The size of the square morphological operator. */
  int m_morphKernelSize;

  /** The number of rows in the image matrix. */
  int m_rows;

  /** The settings to use for InfiniTAM. */
  Settings_CPtr m_settings;

  /** A thresholded version of the raw depth image captured from the camera in which parts of the scene > 2m away have been masked out. */
  ITMFloatImage_Ptr m_thresholdedRawDepth;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief An instance of this class may be used to identify those pixels which are touching a surface.
   *
   * \param imgSize   TODO
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

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief TODO
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
   * \param candidateComponents The IDs of components in the connected component image that denote promising touch regions.
   * \param diffRawRaycastInMm  An image in which each pixel is the absolute difference in mm between the current and raycasted depths.
   * \return                    The ID of the best candidate component.
   */
  int pick_best_candidate_component(const af::array& candidateComponents, const af::array& diffRawRaycastInMm);

  /**
   * \brief Prepares a thresholded version of the raw depth image and a depth raycast ready for movement detection.
   *
   * \param camera        The camera from which the scene is being rendered.
   * \param rawDepth      The raw depth image from the camera.
   * \param renderState   The render state corresponding to the camera.
   */
  void prepare_inputs(const rigging::MoveableCamera_CPtr& camera, const ITMFloatImage_CPtr& rawDepth, const RenderState_CPtr& renderState);

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
