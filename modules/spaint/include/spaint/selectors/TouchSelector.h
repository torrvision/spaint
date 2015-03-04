/**
 * spaint: TouchSelector.h
 */

#ifndef H_SPAINT_TOUCHSELECTOR
#define H_SPAINT_TOUCHSELECTOR

#include "imageprocessing/interface/ImageProcessing.h"
#include "PickingSelector.h"
#include "../touch/TouchState.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to select a cube of voxels in the scene using touch.
 */
class TouchSelector : public PickingSelector
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<ITMFloatImage> FloatImage_Ptr;

  //#################### PRIVATE VARIABLES #################### 
private:
  /** An image in which each pixel is the difference between the currentandraycasted depth. */
  mutable FloatImage_Ptr m_diffRawRaycast;

  /** Multiplatform image processing tools. */
  boost::shared_ptr<const ImageProcessing> m_imageProcessor;

  /** An image into which to store the depth calculation of the currently visible scene from the camera. */
  mutable FloatImage_Ptr m_raycastedDepthResult;

  /** An instance of the current state of touching. */
  mutable TouchState m_touchState;

  //#################### CONSTRUCTORS #################### 
public:
  /*
   * \brief Constructs a touch selector.
   *
   * \param settings  The settings to use for InfiniTAM.
   */
  explicit TouchSelector(const Settings_CPtr& settings);

  //#################### PUBLIC MEMBER FUNCTIONS #################### 
public:
  /** Override */
  virtual Selection_CPtr get_selection() const;

  /**
   * \brief Gets a touch selection.
   */
  void touch_pipeline() const;

  /** Override */
  virtual void update(const InputState& inputState, const RenderState_CPtr& renderState);
};

}

#endif

