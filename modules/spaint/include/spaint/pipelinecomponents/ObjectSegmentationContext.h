/**
 * spaint: ObjectSegmentationContext.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_OBJECTSEGMENTATIONCONTEXT
#define H_SPAINT_OBJECTSEGMENTATIONCONTEXT

#include <map>

#include <tvgutil/filesystem/SequentialPathGenerator.h>

#include "../segmentation/Segmenter.h"
#include "../slamstate/SLAMState.h"
#include "../visualisation/VisualisationGenerator.h"

namespace spaint {

/**
 * \brief An instance of a class deriving from this one provides the shared context needed by object segmentation components.
 */
class ObjectSegmentationContext
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The overlay image (if any) to render whilst performing segmentation in different scenes. */
  std::map<std::string,ITMUChar4Image_CPtr> m_segmentationImages;

  /** The path generator for the current segmentation video (if any). */
  boost::optional<tvgutil::SequentialPathGenerator> m_segmentationPathGenerator;

  /** The segmenter. */
  Segmenter_Ptr m_segmenter;

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the object segmentation context.
   */
  virtual ~ObjectSegmentationContext();

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  virtual const std::string& get_resources_dir() const = 0;
  virtual Settings_CPtr get_settings(const std::string& sceneID) const = 0;
  virtual const SLAMState_Ptr& get_slam_state(const std::string& sceneID) = 0;
  virtual VisualisationGenerator_CPtr get_visualisation_generator() const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the overlay image (if any) to render whilst performing segmentation in the specified scene.
   *
   * \param sceneID The ID of the scene in which segmentation is being performed.
   * \return        The overlay image (if any) to render whilst performing segmentation.
   */
  virtual ITMUChar4Image_CPtr get_segmentation_image(const std::string& sceneID) const;

  /**
   * \brief Gets the path generator for the current segmentation video (if any).
   *
   * \return  The path generator for the current segmentation video (if any).
   */
  virtual boost::optional<tvgutil::SequentialPathGenerator>& get_segmentation_path_generator();

  /**
   * \brief Gets the segmenter.
   *
   * \return  The segmenter.
   */
  virtual const Segmenter_Ptr& get_segmenter() const;

  /**
   * \brief Sets the overlay image (if any) to render whilst performing segmentation in the specified scene.
   *
   * \param sceneID           The ID of the scene in which segmentation is being performed.
   * \param segmentationImage The overlay image (if any) to render whilst performing segmentation in the specified scene.
   */
  virtual void set_segmentation_image(const std::string& sceneID, const ITMUChar4Image_CPtr& segmentationImage);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Sets the segmenter.
   *
   * \param segmenter The segmenter.
   */
  void set_segmenter(const Segmenter_Ptr& segmenter);

  //#################### FRIENDS ####################

  friend class ObjectSegmentationComponent;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<ObjectSegmentationContext> ObjectSegmentationContext_Ptr;

}

#endif
