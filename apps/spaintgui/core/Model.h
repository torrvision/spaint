/**
 * spaintgui: Model.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINTGUI_MODEL
#define H_SPAINTGUI_MODEL

#include <spaint/pipelinecomponents/PropagationContext.h>
#include <spaint/pipelinecomponents/SemanticSegmentationContext.h>
#include <spaint/pipelinecomponents/SLAMContext.h>
#include <spaint/pipelinecomponents/SmoothingContext.h>
#include <spaint/selectiontransformers/interface/SelectionTransformer.h>
#include <spaint/selectors/Selector.h>
#include <spaint/util/LabelManager.h>

/**
 * \brief An instance of this class represents our model of the spaint scenario.
 *
 * The scenario we model is one of reconstructing a scene from a series of RGB-D images with known (tracked) pose,
 * and labelling it interactively using various user input modalities.
 */
class Model
: public spaint::PropagationContext,
  public spaint::SemanticSegmentationContext,
  public spaint::SLAMContext,
  public spaint::SmoothingContext
{
  //#################### TYPEDEFS ####################
public:
  typedef boost::shared_ptr<ORUtils::MemoryBlock<spaint::SpaintVoxel::PackedLabel> > PackedLabels_Ptr;
  typedef boost::shared_ptr<const ORUtils::MemoryBlock<spaint::SpaintVoxel::PackedLabel> > PackedLabels_CPtr;
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;
  typedef ITMLib::ITMScene<spaint::SpaintVoxel,ITMVoxelIndex> Scene;
  typedef boost::shared_ptr<Scene> Scene_Ptr;
  typedef boost::shared_ptr<const Scene> Scene_CPtr;
  typedef spaint::Selector::Selection Selection;
  typedef boost::shared_ptr<const Selection> Selection_CPtr;
  typedef boost::shared_ptr<const ITMLib::ITMLibSettings> Settings_CPtr;
  typedef boost::shared_ptr<ITMLib::ITMTrackingState> TrackingState_Ptr;
  typedef boost::shared_ptr<const ITMLib::ITMTrackingState> TrackingState_CPtr;
  typedef boost::shared_ptr<ITMLib::ITMView> View_Ptr;
  typedef boost::shared_ptr<const ITMLib::ITMView> View_CPtr;
  typedef ITMLib::ITMVisualisationEngine<spaint::SpaintVoxel,ITMVoxelIndex> VisualisationEngine;
  typedef boost::shared_ptr<VisualisationEngine> VisualisationEngine_Ptr;
  typedef boost::shared_ptr<const VisualisationEngine> VisualisationEngine_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The label manager. */
  spaint::LabelManager_Ptr m_labelManager;

  /** The path to the resources directory. */
  std::string m_resourcesDir;

  /** The selection transformer to use. */
  spaint::SelectionTransformer_Ptr m_selectionTransformer;

  /** The selector to use for selecting voxels in the scene. */
  spaint::Selector_Ptr m_selector;

  /** The semantic label to use for manually labelling the scene. */
  spaint::SpaintVoxel::Label m_semanticLabel;

  /** The settings to use for InfiniTAM. */
  Settings_CPtr m_settings;

  /** The InfiniTAM engine used for raycasting the scene. */
  VisualisationEngine_Ptr m_visualisationEngine;

  /** The voxel marker (used to apply semantic labels to voxels in the scene). */
  spaint::VoxelMarker_CPtr m_voxelMarker;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a model.
   *
   * \param settings        The settings to use for InfiniTAM.
   * \param resourcesDir    The path to the resources directory.
   * \param labelManager    The label manager.
   */
  Model(const Settings_CPtr& settings, const std::string& resourcesDir, const spaint::LabelManager_Ptr& labelManager);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Clears the semantic labels of some or all of voxels in the scene.
   *
   * \param settings  The settings to use for the label-clearing operation.
   */
  virtual void clear_labels(spaint::ClearingSettings settings);

  /**
   * \brief Gets the label manager.
   *
   * \return  The label manager.
   */
  virtual const spaint::LabelManager_Ptr& get_label_manager();

  /**
   * \brief Gets the label manager.
   *
   * \return  The label manager.
   */
  virtual spaint::LabelManager_CPtr get_label_manager() const;

  /**
   * \brief Gets the path to the resources directory.
   *
   * \return The path to the resources directory.
   */
  virtual const std::string& get_resources_dir() const;

  /** Override */
  virtual const Scene_Ptr& get_scene();

  /** Override */
  virtual Scene_CPtr get_scene() const;

  /**
   * \brief Gets the voxels in the scene (if any) that were selected the last time the current selector was updated.
   *
   * \return  The voxels in the scene (if any) that were selected the last time the current selector was updated.
   */
  virtual Selection_CPtr get_selection() const;

  /**
   * \brief Gets the selection transformer that is currently being used.
   *
   * \return  The selection transformer that is currently being used.
   */
  virtual spaint::SelectionTransformer_CPtr get_selection_transformer() const;

  /**
   * \brief Gets the selector that is currently being used to select voxels in the scene.
   *
   * \return  The selector that is currently being used to select voxels in the scene.
   */
  virtual spaint::Selector_CPtr get_selector() const;

  /**
   * \brief Gets the semantic label to use for manually labelling the scene.
   *
   * \return  The semantic label to use for manually labelling the scene.
   */
  virtual spaint::SpaintVoxel::Label get_semantic_label() const;

  /**
   * \brief Gets the settings to use for InfiniTAM.
   *
   * \return  The settings to use for InfiniTAM.
   */
  virtual const Settings_CPtr& get_settings() const;

  /**
   * \brief Gets the InfiniTAM engine used for raycasting the scene.
   *
   * \return  The InfiniTAM engine used for raycasting the scene.
   */
  virtual VisualisationEngine_CPtr get_visualisation_engine() const;

  /**
   * \brief Marks a selection of voxels in the scene with the specified semantic label.
   *
   * \param selection The selection of voxels.
   * \param label     The semantic label with which to mark the voxels.
   * \param scene     The scene.
   * \param mode      The marking mode.
   * \param oldLabels An optional memory block into which to store the old semantic labels of the voxels being marked.
   */
  virtual void mark_voxels(const Selection_CPtr& selection, spaint::SpaintVoxel::PackedLabel label, const Scene_Ptr& scene,
                           spaint::MarkingMode mode, const PackedLabels_Ptr& oldLabels = PackedLabels_Ptr());

  /**
   * \brief Marks a selection of voxels in the scene with the specified semantic labels.
   *
   * \param selection The selection of voxels.
   * \param labels    The semantic labels with which to mark the voxels (one per voxel).
   * \param scene     The scene.
   * \param mode      The marking mode.
   */
  virtual void mark_voxels(const Selection_CPtr& selection, const PackedLabels_CPtr& labels, const Scene_Ptr& scene, spaint::MarkingMode mode);

  /**
   * \brief Sets the semantic label to use for manually labelling the scene.
   *
   * \param semanticLabel The semantic label to use for manually labelling the scene.
   */
  virtual void set_semantic_label(spaint::SpaintVoxel::Label semanticLabel);

  /**
   * \brief Allows the user to change selector or update the current selector.
   *
   * \param inputState      The current input state.
   * \param renderState     The render state corresponding to the camera from which the scene is being viewed.
   * \param renderingInMono A flag indicating whether or not the scene is currently being rendered in mono.
   */
  virtual void update_selector(const tvginput::InputState& inputState, const RenderState_CPtr& renderState, bool renderingInMono);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<Model> Model_Ptr;
typedef boost::shared_ptr<const Model> Model_CPtr;

#endif
