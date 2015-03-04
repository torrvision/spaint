/**
 * spaint: TouchSelector.h
 */

#ifndef H_SPAINT_TOUCHSELECTOR
#define H_SPAINT_TOUCHSELECTOR

#include "PickingSelector.h"

#include "imageprocessing/interface/ImageProcessing.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to select a cube of voxels in the scene using touch.
 */
class TouchSelector : public PickingSelector
{
  //#################### TYPEDEFS ####################
protected:
  typedef boost::shared_ptr<ITMFloatImage> FloatImage_Ptr;

  //#################### PRIVATE VARIABLES #################### 
private:
  /** An image into which to store the depth calculation of the currently visible scene from the camera. */
  mutable FloatImage_Ptr m_raycastedDepthResult;

  /** An image in which each pixel is the difference between the currentandraycasted depth. */
  mutable FloatImage_Ptr m_diffRawRaycast;

  /** Multiplatform image processing tools. */
  boost::shared_ptr<const ImageProcessing> m_imageProcessor;

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
};

}

#endif

