/**
 * spaint: TouchDetector.h
 */

#ifndef H_SPAINT_TOUCHDETECTOR
#define H_SPAINT_TOUCHDETECTOR

#include <arrayfire.h>

#include <ITMLib/Objects/ITMRenderState.h>

#include <rigging/SimpleCamera.h>

#include "../imageprocessing/interface/ImageProcessor.h"
#include "../visualisers/interface/DepthVisualiser.h"
#include "TouchState.h"

//#define DEBUG_TOUCH_VERBOSE
//#define DEBUG_TOUCH_DISPLAY

namespace spaint {

/**
 * \brief An instance of this class can be used to detect a touch interaction.
 */
class TouchDetector
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<af::array> AFImage_Ptr;
  typedef boost::shared_ptr<ITMFloatImage> FloatImage_Ptr;
  typedef boost::shared_ptr<const ITMLib::Objects::ITMRenderState> RenderState_CPtr;
  typedef std::vector<Eigen::Vector2i> Points;
  typedef boost::shared_ptr<Points> Points_Ptr;
  typedef boost::shared_ptr<const Points> Points_CPtr;

#ifdef DEBUG_TOUCH_DISPLAY
  //#################### PRIVATE DEBUGGING VARIABLES ####################
private:
  /** The delay between processing frames (0 = pause). */
  int m_debugDelayms;

  /** The lower depth threshold value in millimeters needed to the opencv trackbar. */
  int m_depthLowerThresholdmm;
#endif

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
  AFImage_Ptr m_diffRawRaycast;

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
  FloatImage_Ptr m_rawDepthCopy;

  /** An image into which to store the depth of the currently visible scene from the camera. */
  FloatImage_Ptr m_raycastedDepthResult;

  /** The number of rows in the image matrix. */
  int m_rows;

  /** An array in which to store a binary image resulting fro a threshold operation. */
  af::array m_thresholded;

  /** An instance of the current touch state. */
  TouchState m_touchState;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief An instance of this class may be used to identify those pixels which are touching a surface.
   */
  explicit TouchDetector(const Vector2i& imgSize);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Perform image processing routines on the raw and raycasted depth images in order to set the touch state.
   *
   * \param renderState   The render state.
   * \param camera        The camera.
   * \param voxelSize     The scene voxel size.
   * \param rawDepth      The raw depth image from the camera.
   */
  void run_touch_detector_on_frame(const RenderState_CPtr& renderState, const rigging::MoveableCamera_CPtr camera, float voxelSize, const ITMFloatImage *rawDepth);

  /**
   * \brief Gets the touch state.
   *
   * \return  The touch state.
   */
  const TouchState& get_touch_state() const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Find image regions which differ from the raw and raycasted depth images.
   *
   * \param renderState   The render state.
   * \param camera        The camera.
   * \param voxelSize     The scene voxel size.
   * \param rawDepth      The raw depth image from the camera.
   */
  void calculate_binary_difference_image(const RenderState_CPtr& renderState, const rigging::MoveableCamera_CPtr camera, float voxelSize, const ITMFloatImage *rawDepth);

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

  /**
   * \brief Convert an array to unsigned char truncating values outside the range 0-255.
   *
   * \param array   The input array.
   * \return        The truncated and converted array.
   */
  af::array truncate_to_unsigned_char(const af::array& array);

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
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
