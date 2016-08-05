/**
 * spaint: MotionBasedObjectSegmenter.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_MOTIONBASEDOBJECTSEGMENTER
#define H_SPAINT_MOTIONBASEDOBJECTSEGMENTER

#include "ColourAppearanceModel.h"
#include "ObjectSegmenter.h"
#include "../touch/TouchDetector.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to segment a moving object based on its motion with respect to the scene.
 */
class MotionBasedObjectSegmenter : public ObjectSegmenter
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ITMFloatImage> ITMFloatImage_CPtr;
  typedef boost::shared_ptr<const ITMLib::ITMLibSettings> ITMSettings_CPtr;
  typedef boost::shared_ptr<ITMUCharImage> ITMUCharImage_Ptr;
  typedef boost::shared_ptr<const ITMUChar4Image> ITMUChar4Image_CPtr;
  typedef boost::shared_ptr<const ITMLib::ITMView> View_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The colour appearance model to use to separate the user's hand from any object it's holding. */
  ColourAppearanceModel_Ptr m_handAppearanceModel;

  /** TODO */
  ITMUCharImage_Ptr m_objectMask;

  /** The touch detector to use to get the initial difference mask. */
  mutable TouchDetector_Ptr m_touchDetector;

  /** TODO */
  View_CPtr m_view;

  //#################### CONSTRUCTORS ####################
public:
  MotionBasedObjectSegmenter(const ITMSettings_CPtr& itmSettings, const TouchSettings_Ptr& touchSettings, const View_CPtr& view);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual ITMUCharImage_CPtr get_mask() const;

  /** Override */
  virtual void reset();

  /** Override */
  virtual ITMUCharImage_CPtr segment(const ORUtils::SE3Pose& pose, const RenderState_CPtr& renderState) const;

  /** Override */
  virtual ITMUChar4Image_Ptr train(const ORUtils::SE3Pose& pose, const RenderState_CPtr& renderState);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief TODO
   */
  ITMUCharImage_CPtr make_change_mask(const ITMFloatImage_CPtr& depthInput, const ORUtils::SE3Pose& pose, const RenderState_CPtr& renderState) const;

  /**
   * \brief TODO
   */
  ITMUCharImage_CPtr make_touch_mask(const ITMFloatImage_CPtr& depthInput, const ORUtils::SE3Pose& pose, const RenderState_CPtr& renderState) const;
};

}

#endif
