/**
 * spaint: TouchDetector.h
 */

#ifndef H_SPAINT_TOUCHDETECTOR
#define H_SPAINT_TOUCHDETECTOR

#include <ITMLib/Objects/ITMRenderState.h>

#include <rigging/SimpleCamera.h>

#include "../imageprocessing/interface/ImageProcessing.h"
#include "TouchState.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to detect a touch interaction.
 */
class TouchDetector
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<ITMFloatImage> FloatImage_Ptr;
  typedef boost::shared_ptr<ITMLib::Objects::ITMRenderState> RenderState_Ptr;

  //#################### PRIVATE VARIABLES #################### 
private:
  /** An image in which each pixel is the difference between the currentandraycasted depth. */
  FloatImage_Ptr m_diffRawRaycast;

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
  TouchDetector();

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
   * \brief Gets a depth raycast of the scene.
   *
   * \param output      The location into which to store the depth values.
   * \param renderState The current render state.
   * \param camera      The camera from which to raycast the scene.
   * \param voxelSize   The size of a voxel in the scene, usually in meters.
   */
  void generate_depth_raycast(const FloatImage_Ptr& output, const RenderState_Ptr& renderState, const rigging::SimpleCamera_Ptr camera, float voxelSize) const;
};

}

#endif

