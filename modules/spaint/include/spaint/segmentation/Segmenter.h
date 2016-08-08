/**
 * spaint: Segmenter.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SEGMENTER
#define H_SPAINT_SEGMENTER

#include <ITMLib/Objects/RenderStates/ITMRenderState.h>
#include <ITMLib/Objects/Views/ITMView.h>

#include <ORUtils/SE3Pose.h>

#include "../util/ITMImagePtrTypes.h"

namespace spaint {

/**
 * \brief An instance of a class deriving from this one can be used to segment a target from the current view of the scene.
 */
class Segmenter
{
  //#################### TYPEDEFS ####################
protected:
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;
  typedef boost::shared_ptr<const ITMLib::ITMView> View_CPtr;

  //#################### PROTECTED VARIABLES ####################
protected:
  /** The most recent target mask produced by the segmentation process. */
  ITMUCharImage_Ptr m_targetMask;

  /** The current view of the scene. */
  View_CPtr m_view;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a segmenter.
   *
   * \param view  The current view of the scene.
   */
  explicit Segmenter(const View_CPtr& view);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the segmenter.
   */
  virtual ~Segmenter();

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Resets the segmenter.
   */
  virtual void reset() = 0;

  /**
   * \brief Segments the target from the current view of the scene.
   *
   * \param pose        The camera pose from which the scene is being viewed.
   * \param renderState The render state corresponding to the camera.
   * \return            The target mask produced by the segmentation process.
   */
  virtual ITMUCharImage_CPtr segment(const ORUtils::SE3Pose& pose, const RenderState_CPtr& renderState) const = 0;

  /**
   * \brief Trains the segmenter.
   *
   * \param pose        The camera pose from which the scene is being viewed.
   * \param renderState The render state corresponding to the camera.
   * \return            A visualisation of the training process to enable the user to see what's going on.
   */
  virtual ITMUChar4Image_Ptr train(const ORUtils::SE3Pose& pose, const RenderState_CPtr& renderState) = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the most recent target mask produced by the segmentation process.
   *
   * \return  The most recent target mask produced by the segmentation process.
   */
  ITMUCharImage_CPtr get_target_mask() const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<Segmenter> Segmenter_Ptr;

}

#endif
