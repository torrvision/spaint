/**
 * spaint: SemanticSegmentationContext.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SEMANTICSEGMENTATIONCONTEXT
#define H_SPAINT_SEMANTICSEGMENTATIONCONTEXT

#include "../markers/shared/VoxelMarker_Settings.h"
#include "../selectors/Selector.h"
#include "../util/LabelManager.h"
#include "../util/SLAMState.h"

namespace spaint {

/**
 * \brief An instance of a class deriving from this one provides the shared context needed by a semantic segmentation component.
 */
class SemanticSegmentationContext
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ORUtils::MemoryBlock<SpaintVoxel::PackedLabel> > PackedLabels_CPtr;
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
  virtual const std::string& get_resources_dir() const = 0;
  virtual Selector_CPtr get_selector() const = 0;
  virtual const Settings_CPtr& get_settings() const = 0;
  virtual const SLAMState_Ptr& get_slam_state(const std::string& sceneID) = 0;
  virtual void mark_voxels(const std::string& sceneID, const Selection_CPtr& selection, const PackedLabels_CPtr& labels, MarkingMode mode) = 0;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<SemanticSegmentationContext> SemanticSegmentationContext_Ptr;

}

#endif
