/**
 * spaintgui: Model.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINTGUI_MODEL
#define H_SPAINTGUI_MODEL

#include <spaint/markers/interface/VoxelMarker.h>
#include <spaint/pipelinecomponents/CollaborativeContext.h>
#include <spaint/pipelinecomponents/ObjectSegmentationContext.h>
#include <spaint/pipelinecomponents/PropagationContext.h>
#include <spaint/pipelinecomponents/SemanticSegmentationContext.h>
#include <spaint/pipelinecomponents/SLAMContext.h>
#include <spaint/pipelinecomponents/SmoothingContext.h>
#include <spaint/selectiontransformers/interface/SelectionTransformer.h>
#include <spaint/selectors/Selector.h>
#include <spaint/util/LabelManager.h>
#include <spaint/visualisation/VisualisationGenerator.h>

/**
 * \brief An instance of this class represents our model of the spaint scenario.
 *
 * The scenario we model is one of reconstructing a scene from a series of RGB-D images with known (tracked) pose,
 * and labelling it interactively using various user input modalities.
 */
class Model
: public spaint::CollaborativeContext,
  public spaint::ObjectSegmentationContext,
  public spaint::PropagationContext,
  public spaint::SemanticSegmentationContext,
  public spaint::SLAMContext,
  public spaint::SmoothingContext
{
  //#################### TYPEDEFS ####################
public:
  typedef boost::shared_ptr<ORUtils::MemoryBlock<spaint::SpaintVoxel::PackedLabel> > PackedLabels_Ptr;
  typedef boost::shared_ptr<const ORUtils::MemoryBlock<spaint::SpaintVoxel::PackedLabel> > PackedLabels_CPtr;
  typedef spaint::Selector::Selection Selection;
  typedef boost::shared_ptr<const Selection> Selection_CPtr;
  typedef boost::shared_ptr<const ITMLib::ITMSurfelVisualisationEngine<spaint::SpaintSurfel> > SurfelVisualisationEngine_CPtr;
  typedef ITMLib::ITMVisualisationEngine<spaint::SpaintVoxel,ITMVoxelIndex> VoxelVisualisationEngine;
  typedef boost::shared_ptr<VoxelVisualisationEngine> VoxelVisualisationEngine_Ptr;
  typedef boost::shared_ptr<const VoxelVisualisationEngine> VoxelVisualisationEngine_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The label manager. */
  spaint::LabelManager_Ptr m_labelManager;

  /** The ID of the fiducial (if any) from which to obtain the Leap Motion controller's coordinate frame. */
  std::string m_leapFiducialID;

  /** The remote mapping server (if any). */
  itmx::MappingServer_Ptr m_mappingServer;

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

  /** The InfiniTAM engine used for rendering a surfel scene. */
  SurfelVisualisationEngine_CPtr m_surfelVisualisationEngine;

  /** The visualisation generator that is used to render a scene. */
  spaint::VisualisationGenerator_Ptr m_visualisationGenerator;

  /** The voxel marker (used to apply semantic labels to voxels in the scene). */
  spaint::VoxelMarker_CPtr m_voxelMarker;

  /** The InfiniTAM engine used for rendering a voxel scene. */
  VoxelVisualisationEngine_Ptr m_voxelVisualisationEngine;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a model.
   *
   * \param settings        The settings to use for InfiniTAM.
   * \param resourcesDir    The path to the resources directory.
   * \param maxLabelCount   The maximum number of labels that can be in use.
   * \param mappingServer   The remote mapping server (if any).
   */
  Model(const Settings_CPtr& settings, const std::string& resourcesDir, size_t maxLabelCount, const itmx::MappingServer_Ptr& mappingServer);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Clears the semantic labels of some or all of voxels in the specified scene.
   *
   * \param sceneID   The scene ID.
   * \param settings  The settings to use for the label-clearing operation.
   */
  virtual void clear_labels(const std::string& sceneID, spaint::ClearingSettings settings);

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
   * \brief Gets the remote mapping server (if any).
   *
   * \return  The remote mapping server (if any).
   */
  virtual const itmx::MappingServer_Ptr& get_mapping_server();

  /**
   * \brief Gets the remote mapping server (if any).
   *
   * \return  The remote mapping server (if any).
   */
  virtual itmx::MappingServer_CPtr get_mapping_server() const;

  /**
   * \brief Gets the path to the resources directory.
   *
   * \return The path to the resources directory.
   */
  virtual const std::string& get_resources_dir() const;

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
   * \brief Gets the InfiniTAM engine used for rendering a surfel scene.
   *
   * \return  The InfiniTAM engine used for rendering a surfel scene.
   */
  virtual SurfelVisualisationEngine_CPtr get_surfel_visualisation_engine() const;

  /**
   * \brief Gets the visualisation generator that is used to render a scene.
   *
   * \return  The visualisation generator that is used to render a scene.
   */
  virtual spaint::VisualisationGenerator_CPtr get_visualisation_generator() const;

  /**
   * \brief Gets the InfiniTAM engine used for rendering a voxel scene.
   *
   * \return  The InfiniTAM engine used for rendering a voxel scene.
   */
  virtual VoxelVisualisationEngine_CPtr get_voxel_visualisation_engine() const;

  /**
   * \brief Marks a selection of voxels in a scene with the specified semantic label.
   *
   * \param sceneID   The scene ID.
   * \param selection The selection of voxels.
   * \param label     The semantic label with which to mark the voxels.
   * \param mode      The marking mode.
   * \param oldLabels An optional memory block into which to store the old semantic labels of the voxels being marked.
   */
  virtual void mark_voxels(const std::string& sceneID, const Selection_CPtr& selection, spaint::SpaintVoxel::PackedLabel label,
                           spaint::MarkingMode mode, const PackedLabels_Ptr& oldLabels = PackedLabels_Ptr());

  /**
   * \brief Marks a selection of voxels in a scene with the specified semantic labels.
   *
   * \param sceneID   The scene ID.
   * \param selection The selection of voxels.
   * \param labels    The semantic labels with which to mark the voxels (one per voxel).
   * \param mode      The marking mode.
   */
  virtual void mark_voxels(const std::string& sceneID, const Selection_CPtr& selection, const PackedLabels_CPtr& labels, spaint::MarkingMode mode);

  /**
   * \brief Sets the ID of the fiducial (if any) from which to obtain the Leap Motion controller's coordinate frame.
   *
   * \param leapFiducialID  The ID of the fiducial (if any) from which to obtain the Leap Motion controller's coordinate frame.
   */
  void set_leap_fiducial_id(const std::string& leapFiducialID);

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
   * \param slamState       The SLAM state of the scene being viewed.
   * \param renderState     The voxel render state corresponding to the camera from which the scene is being viewed.
   * \param renderingInMono A flag indicating whether or not the scene is currently being rendered in mono.
   */
  virtual void update_selector(const tvginput::InputState& inputState, const spaint::SLAMState_CPtr& slamState, const VoxelRenderState_CPtr& renderState, bool renderingInMono);

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the ID for the world scene.
   *
   * \return  The ID for the world scene.
   */
  static std::string get_world_scene_id();

  //#################### DISAMBIGUATORS ####################
public:
  virtual orx::RefiningRelocaliser_Ptr& get_relocaliser(const std::string& sceneID);
  virtual orx::RefiningRelocaliser_CPtr get_relocaliser(const std::string& sceneID) const;
  virtual const std::vector<std::string>& get_scene_ids() const;
  virtual const spaint::SLAMState_Ptr& get_slam_state(const std::string& sceneID);
  virtual spaint::SLAMState_CPtr get_slam_state(const std::string& sceneID) const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<Model> Model_Ptr;
typedef boost::shared_ptr<const Model> Model_CPtr;

#endif
