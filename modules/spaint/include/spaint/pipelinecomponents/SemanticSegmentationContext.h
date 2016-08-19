/**
 * spaint: SemanticSegmentationContext.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SEMANTICSEGMENTATIONCONTEXT
#define H_SPAINT_SEMANTICSEGMENTATIONCONTEXT

#include "../markers/interface/VoxelMarker.h"
#include "../selectors/Selector.h"
#include "../util/LabelManager.h"
#include "../util/SpaintVoxel.h"

namespace spaint {

/**
 * \brief An instance of a class deriving from this one provides the shared context needed by a semantic segmentation component.
 */
class SemanticSegmentationContext
{
  //#################### TYPEDEFS ####################
private:
  typedef ITMLib::ITMScene<SpaintVoxel,ITMVoxelIndex> Scene;
  typedef boost::shared_ptr<Scene> Scene_Ptr;

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the semantic segmentation context.
   */
  virtual ~SemanticSegmentationContext() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the label manager.
   *
   * \return  The label manger.
   */
  virtual const LabelManager_Ptr& get_label_manager() = 0;

  /**
   * \brief Gets the reconstructed scene.
   *
   * \return  The reconstructed scene.
   */
  virtual const Scene_Ptr& get_scene() = 0;

  /**
   * \brief TODO
   */
  virtual Selector_CPtr get_selector() const = 0;

  /**
   * \brief TODO
   */
  virtual const VoxelMarker_CPtr& get_voxel_marker() const = 0;
};

}

#endif
