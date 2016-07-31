/**
 * spaint: ObjectSegmenter.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_OBJECTSEGMENTER
#define H_SPAINT_OBJECTSEGMENTER

#include <boost/shared_ptr.hpp>

#include <ITMLib/Objects/RenderStates/ITMRenderState.h>
#include <ITMLib/Utils/ITMImageTypes.h>

#include <ORUtils/SE3Pose.h>

namespace spaint {

/**
 * \brief An instance of a class deriving from this one can be used to segment an object from the live camera input.
 */
class ObjectSegmenter
{
  //#################### TYPEDEFS ####################
protected:
  typedef boost::shared_ptr<const ITMUCharImage> ITMUCharImage_CPtr;
  typedef boost::shared_ptr<ITMUChar4Image> ITMUChar4Image_Ptr;
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the object segmenter.
   */
  virtual ~ObjectSegmenter() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  virtual ITMUCharImage_CPtr get_mask() const = 0;

  /**
   * \brief Resets the colour appearance model used to separate the user's hand from any object it's holding.
   */
  virtual void reset_hand_model() = 0;

  /**
   * \brief TODO
   *
   * \param pose        TODO
   * \param renderState TODO
   * \return            TODO
   */
  virtual ITMUCharImage_CPtr segment_object(const ORUtils::SE3Pose& pose, const RenderState_CPtr& renderState) const = 0;

  /**
   * \brief Trains the colour appearance model used to separate the user's hand from any object it's holding.
   *
   * \param view        TODO
   * \param pose        TODO
   * \param renderState TODO
   * \return            TODO
   */
  virtual ITMUChar4Image_Ptr train_hand_model(const ORUtils::SE3Pose& pose, const RenderState_CPtr& renderState) = 0;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<ObjectSegmenter> ObjectSegmenter_Ptr;

}

#endif
