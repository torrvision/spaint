/**
 * spaint: TouchDetector.h
 */

#ifndef H_SPAINT_TOUCHDETECTOR
#define H_SPAINT_TOUCHDETECTOR

#include <arrayfire.h>

#include <ITMLib/Objects/ITMRenderState.h>

#include <rigging/SimpleCamera.h>

#include "../imageprocessing/interface/ImageProcessing.h"
#include "../visualisers/interface/DepthCalculator.h"
#include "TouchState.h"

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
  typedef boost::shared_ptr<ITMLib::Objects::ITMRenderState> RenderState_Ptr;

  //#################### PRIVATE VARIABLES #################### 
private:
  /** The depth calculator. */
  boost::shared_ptr<const DepthCalculator> m_depthCalculator;

  /** An image in which each pixel is the difference between the currentandraycasted depth. */
  FloatImage_Ptr m_diffRawRaycast;
  AFImage_Ptr m_workspace;

  /** Multiplatform image processing tools. */
  boost::shared_ptr<const ImageProcessing> m_imageProcessor;

  /** An image into which to store the depth calculation of the currently visible scene from the camera. */
  FloatImage_Ptr m_raycastedDepthResult;

  /** An instance of the current state of touching. */
  TouchState m_touchState;

  //#################### CONSTRUCTORS #################### 
public:
  /**
   * \brief TODO.
   */
  TouchDetector(const Vector2i& imgSize);

  //#################### PUBLIC MEMBER FUNCTIONS #################### 
public:
  /**
   * \brief TODO.
   */
  void run_touch_detector_on_frame(const RenderState_Ptr& renderState, const rigging::SimpleCamera_Ptr camera, float voxelSize, ITMFloatImage *rawDepth) const;

  /**
   * \brief TODO.
   */
  const TouchState& get_touch_state() const;

  //#################### PRIVATE MEMBER FUNCTIONS #################### 
private:
  /**
   * \brief An implementation of the touch detection in opencv for cpu only.
   */
  void opencv_cpu_pipeline(const FloatImage_Ptr& rawDiff) const;
};

}

#endif

