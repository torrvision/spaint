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
#include "TouchState.h"

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
  /** The number of columns in the image matrix, (width). */
  int m_cols;

  /** The image holding the connedted components. */
  af::array m_connectedComponents;

  /** The depth calculator. */
  boost::shared_ptr<const DepthVisualiser> m_depthCalculator;

  /** The threshold below which the raw and raycasted depth is assumed to be equal. */
  float m_depthLowerThreshold;

  /** The threshold above which any difference in the raw and raycasted depth is ignored. */
  float m_depthUpperThreshold;

  /** An image in which each pixel is the absolute difference between the current and raycasted depth. */
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

  /** A copy of the raw depth captured from Kinect. */
  ITMFloatImage_Ptr m_rawDepthCopy;

  /** An image into which to store the depth of the currently visible scene from the camera. */
  ITMFloatImage_Ptr m_raycastedDepthResult;

  /** The number of rows in the image matrix. */
  int m_rows;

  /** The settings to use for InfiniTAM. */
  Settings_CPtr m_settings;

  /** An array in which to store a binary image resulting fro a threshold operation. */
  af::array m_thresholded;

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
   * \brief Perform image processing routines on the raw and raycasted depth images in order to determine the touch state.
   *
   * \param camera        The camera from which the scene is being rendered.
   * \param rawDepth      The raw depth image from the camera.
   * \param renderState   The render state corresponding to the camera (contains the raycasted depth image).
   * \return              The touch state.
   */
  TouchState determine_touch_state(const rigging::MoveableCamera_CPtr& camera, const ITMFloatImage_CPtr& rawDepth, const RenderState_CPtr& renderState);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Find image regions which differ between the raw and raycasted depth images.
   *
   * \param camera        The camera from which the scene is being rendered.
   * \param rawDepth      The raw depth image from the camera.
   * \param renderState   The render state corresponding to the camera (contains the raycasted depth image).
   */
  void calculate_binary_difference_image(const rigging::MoveableCamera_CPtr& camera, const ITMFloatImage_CPtr& rawDepth, const RenderState_CPtr& renderState);

  /**
   * \brief Filter a binary image to remove small and spurious regions.
   */
  void filter_binary_image();

  /**
   * \brief Select the best candidate amongst the good candidates.
   *
   * \param goodCandidates         Image regions which are promising touch regions.
   * \param diffCopyMillimetersU8  A copy of the difference image in millimeters and as an unsigned char.
   * \return                       The best candidate region.
   */
  int find_best_connected_component(const af::array& goodCandidates, const af::array& diffCopyMillimetersU8);

  /**
   * \brief Get the touch points from the best connected component.
   *
   * \param bestCandidate          The id of the best connected component.
   * \param diffCopyMillimetersU8  The diff in millimeters unsigned char.
   * \return                       The touch points.
   */
  Points_CPtr get_touch_points(int bestConnectedComponent, const af::array& diffCopyMillimetersU8);

  /**
   * \brief Select good connected components.
   */
  af::array select_good_connected_components();

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
