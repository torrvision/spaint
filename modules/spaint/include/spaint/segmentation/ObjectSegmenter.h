/**
 * spaint: ObjectSegmenter.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_OBJECTSEGMENTER
#define H_SPAINT_OBJECTSEGMENTER

#include "ColourAppearanceModel.h"
#include "../touch/TouchDetector.h"

namespace spaint {

/**
 * \brief TODO
 */
class ObjectSegmenter
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ITMFloatImage> ITMFloatImage_CPtr;
  typedef boost::shared_ptr<const ITMLib::ITMLibSettings> ITMSettings_CPtr;
  typedef boost::shared_ptr<ITMUCharImage> ITMUCharImage_Ptr;
  typedef boost::shared_ptr<const ITMUCharImage> ITMUCharImage_CPtr;
  typedef boost::shared_ptr<ITMUChar4Image> ITMUChar4Image_Ptr;
  typedef boost::shared_ptr<const ITMUChar4Image> ITMUChar4Image_CPtr;
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;
  typedef boost::shared_ptr<const ITMLib::ITMView> View_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The colour appearance model to use to separate the user's hand from any object it's holding. */
  ColourAppearanceModel_Ptr m_handAppearanceModel;

  /** The touch detector to use to get the initial difference mask. */
  mutable TouchDetector_Ptr m_touchDetector;

  /** TODO */
  View_CPtr m_view;

  //#################### CONSTRUCTORS ####################
public:
  ObjectSegmenter(const ITMSettings_CPtr& itmSettings, const TouchSettings_Ptr& touchSettings, const View_CPtr& view);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Resets the colour appearance model used to separate the user's hand from any object it's holding.
   */
  void reset_hand_model();

  /**
   * \brief TODO
   *
   * \param pose        TODO
   * \param renderState TODO
   * \return            TODO
   */
  ITMUCharImage_Ptr segment_object(const ORUtils::SE3Pose& pose, const RenderState_CPtr& renderState) const;

  /**
   * \brief Trains the colour appearance model used to separate the user's hand from any object it's holding.
   *
   * \param view        TODO
   * \param pose        TODO
   * \param renderState TODO
   * \return            TODO
   */
  ITMUChar4Image_Ptr train_hand_model(const ORUtils::SE3Pose& pose, const RenderState_CPtr& renderState);

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   *
   * \param mask  TODO
   * \param image TODO
   */
  static ITMUChar4Image_Ptr apply_mask(const ITMUCharImage_CPtr& mask, const ITMUChar4Image_CPtr& image);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief TODO
   */
  ITMUCharImage_CPtr make_touch_mask(const ITMFloatImage_CPtr& depthInput, const ORUtils::SE3Pose& pose, const RenderState_CPtr& renderState) const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<ObjectSegmenter> ObjectSegmenter_Ptr;

}

#endif
