/**
 * spaint: ObjectSegmentationComponent.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_OBJECTSEGMENTATIONCOMPONENT
#define H_SPAINT_OBJECTSEGMENTATIONCOMPONENT

#include "ObjectSegmentationContext.h"
#include "../imagesources/SingleRGBDImagePipe.h"
#include "../segmentation/Segmenter.h"

namespace spaint {

/**
 * \brief An instance of this pipeline component can be used to segment objects from the rest of the scene.
 */
class ObjectSegmentationComponent
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The shared context needed for object segmentation. */
  ObjectSegmentationContext_Ptr m_context;

  /** A flag controlling whether or not to write the masked RGB and depth images to the output pipe. */
  bool m_outputEnabled;

  /** The pipe to which to write the masked RGB and depth images resulting from the segmentation process. */
  SingleRGBDImagePipe_Ptr m_outputPipe;

  /** The ID of the scene on which the component should operate. */
  std::string m_sceneID;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an object segmentation component.
   *
   * \param context     The shared context needed for object segmentation.
   * \param sceneID     The ID of the scene on which the component should operate.
   * \param outputPipe  The pipe to which to write the masked RGB and depth images resulting from the segmentation process.
   */
  ObjectSegmentationComponent(const ObjectSegmentationContext_Ptr& context, const std::string& sceneID, const SingleRGBDImagePipe_Ptr& outputPipe = SingleRGBDImagePipe_Ptr());

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Resets the segmenter.
   */
  void reset_segmenter();

  /**
   * \brief Runs the segmentation section of the component.
   *
   * \param renderState The render state associated with the camera position from which to segment the target.
   */
  void run_segmentation(const VoxelRenderState_CPtr& renderState);

  /**
   * \brief Runs the segmentation training section of the component.
   *
   * \param renderState The render state associated with the camera position from which to train the segmenter.
   */
  void run_segmentation_training(const VoxelRenderState_CPtr& renderState);

  /**
   * \brief Toggles whether or not to write the masked RGB and depth images to the output pipe.
   */
  void toggle_output();

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Gets the segmenter.
   *
   * \return  The segmenter.
   */
  const Segmenter_Ptr& get_segmenter() const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<ObjectSegmentationComponent> ObjectSegmentationComponent_Ptr;

}

#endif
