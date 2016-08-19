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
  typedef boost::shared_ptr<const ORUtils::MemoryBlock<spaint::SpaintVoxel::PackedLabel> > PackedLabels_CPtr;
  typedef ITMLib::ITMScene<SpaintVoxel,ITMVoxelIndex> Scene;
  typedef boost::shared_ptr<Scene> Scene_Ptr;
  typedef spaint::Selector::Selection Selection;
  typedef boost::shared_ptr<const Selection> Selection_CPtr;

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the semantic segmentation context.
   */
  virtual ~SemanticSegmentationContext() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  virtual const LabelManager_Ptr& get_label_manager() = 0;
  virtual const Scene_Ptr& get_scene() = 0;
  virtual Selector_CPtr get_selector() const = 0;
  virtual void mark_voxels(const Selection_CPtr& selection, const PackedLabels_CPtr& labels, const Scene_Ptr& scene, spaint::MarkingMode mode) = 0;
};

}

#endif
