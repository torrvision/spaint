/**
 * spaintgui: Interactor.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINTGUI_INTERACTOR
#define H_SPAINTGUI_INTERACTOR

#include <boost/shared_ptr.hpp>

#include <spaint/markers/interface/VoxelMarker.h>
#include <spaint/selectiontransformers/interface/SelectionTransformer.h>
#include <spaint/selectors/Selector.h>

#include "Model.h"

/**
 * \brief An instance of this class can be used to interact with the InfiniTAM scene in an spaint model.
 */
class Interactor
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<ORUtils::MemoryBlock<spaint::SpaintVoxel::PackedLabel> > PackedLabels_Ptr;
  typedef boost::shared_ptr<const ORUtils::MemoryBlock<spaint::SpaintVoxel::PackedLabel> > PackedLabels_CPtr;
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;
  typedef spaint::Selector::Selection Selection;
  typedef boost::shared_ptr<const Selection> Selection_CPtr;
  typedef boost::shared_ptr<spaint::SelectionTransformer> SelectionTransformer_Ptr;
public:
  typedef boost::shared_ptr<const spaint::SelectionTransformer> SelectionTransformer_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The spaint model. */
  Model_Ptr m_model;

  /** The selection transformer to use. */
  SelectionTransformer_Ptr m_selectionTransformer;

  /** The selector to use for selecting voxels in the scene. */
  spaint::Selector_Ptr m_selector;

  /** The semantic label to use for manually labelling the scene. */
  spaint::SpaintVoxel::Label m_semanticLabel;

  /** The voxel marker (used to apply semantic labels to voxels in the scene). */
  spaint::VoxelMarker_CPtr m_voxelMarker;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an interactor that can be used to interact with the InfiniTAM scene.
   *
   * \param model The spaint model.
   */
  explicit Interactor(const Model_Ptr& model);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Clears the semantic labels of some or all of voxels in the scene.
   *
   * \param settings  The settings to use for the label-clearing operation.
   */
  void clear_labels(spaint::ClearingSettings settings);

  /**
   * \brief Gets the voxels in the scene (if any) that were selected the last time the current selector was updated.
   *
   * \return  The voxels in the scene (if any) that were selected the last time the current selector was updated.
   */
  Selection_CPtr get_selection() const;

  /**
   * \brief Gets the selection transformer that is currently being used.
   *
   * \return  The selection transformer that is currently being used.
   */
  SelectionTransformer_CPtr get_selection_transformer() const;

  /**
   * \brief Gets the selector that is currently being used to select voxels in the scene.
   *
   * \return  The selector that is currently being used to select voxels in the scene.
   */
  spaint::Selector_CPtr get_selector() const;

  /**
   * \brief Gets the semantic label that is being used for manually labelling the scene.
   *
   * \return  The semantic label that is being used for manually labelling the scene.
   */
  spaint::SpaintVoxel::Label get_semantic_label() const;

  /**
   * \brief Gets the voxel marker that is being used to apply semantic labels to voxels in the scene.
   *
   * \return  The voxel marker that is being used to apply semantic labels to voxels in the scene.
   */
  const spaint::VoxelMarker_CPtr& get_voxel_marker() const;

  /**
   * \brief Marks a selection of voxels in the scene with the specified semantic label.
   *
   * \param selection The selection of voxels.
   * \param label     The semantic label with which to mark the voxels.
   * \param oldLabels An optional memory block into which to store the old semantic labels of the voxels being marked.
   * \param mode      The marking mode.
   */
  void mark_voxels(const Selection_CPtr& selection, spaint::SpaintVoxel::PackedLabel label, const PackedLabels_Ptr& oldLabels = PackedLabels_Ptr(),
                   spaint::MarkingMode mode = spaint::NORMAL_MARKING);

  /**
   * \brief Marks a selection of voxels in the scene with the specified semantic labels.
   *
   * \param selection The selection of voxels.
   * \param labels    The semantic labels with which to mark the voxels (one per voxel).
   * \param mode      The marking mode.
   */
  void mark_voxels(const Selection_CPtr& selection, const PackedLabels_CPtr& labels, spaint::MarkingMode mode = spaint::NORMAL_MARKING);

  /**
   * \brief Gets whether or not the current selector is active.
   *
   * \return  true, if the current selector is active, or false otherwise.
   */
  bool selector_is_active() const;

  /**
   * \brief Sets the semantic label to use for manually labelling the scene.
   *
   * \param semanticLabel The semantic label to use for manually labelling the scene.
   */
  void set_semantic_label(spaint::SpaintVoxel::Label semanticLabel);

  /**
   * \brief Allows the user to change selector or update the current selector.
   *
   * \param inputState      The current input state.
   * \param renderState     The render state corresponding to the camera from which the scene is being viewed.
   * \param renderingInMono A flag indicating whether or not the scene is currently being rendered in mono.
   */
  void update_selector(const tvginput::InputState& inputState, const RenderState_CPtr& renderState, bool renderingInMono);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<Interactor> Interactor_Ptr;
typedef boost::shared_ptr<const Interactor> Interactor_CPtr;

#endif
