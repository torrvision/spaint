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
#define DEBUG_TOUCH_DISPLAY

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
  /** The minimum image area a candidate touch region must have, expressed as a percentage of the number of pixels in the image. */
  float m_areaPercentageThreshold;

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

  /** The minimum image area required for an image region to be considered as a touch candidate (in number of pixels). */
  int m_minimumAreaThreshold;

  /** The size of the square morphological operator. */
  int m_morphKernelSize;

  /** An image into which to store the depth of the currently visible scene from the camera. */
  FloatImage_Ptr m_raycastedDepthResult;

  /** The number of rows in the image matrix. */
  int m_rows;

  /** An array in which to store a binary image resulting fro a threshold operation. */
  af::array m_thresholded;

  /** An instance of the current touch state. */
  TouchState m_touchState;

  /** An image used to interface with InfiniTAM images. */
  AFImage_Ptr m_workspace;

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
  void run_touch_detector_on_frame(const RenderState_CPtr& renderState, const rigging::MoveableCamera_Ptr camera, float voxelSize, ITMFloatImage *rawDepth);

  /**
   * \brief Gets the touch state.
   *
   * \return  The touch state.
   */
  const TouchState& get_touch_state() const;

#ifdef DEBUG_TOUCH_DISPLAY
  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Display various debugging information.
   *
   * \param rawDepth             The raw depth image from the camera.
   * \param temporaryCandidate   The region which represents the touch interaction.
   */
  void run_debugging_display(ITMFloatImage *rawDepth, const af::array& temporaryCandidate);
#endif
};

}

#endif

