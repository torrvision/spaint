/**
 * spaint: VisualisationGenerator.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_VISUALISATIONGENERATOR
#define H_SPAINT_VISUALISATIONGENERATOR

#include <boost/function.hpp>
#include <boost/optional.hpp>

#include <ITMLib/Engines/Visualisation/Interface/ITMSurfelVisualisationEngine.h>
#include <ITMLib/Engines/Visualisation/Interface/ITMVisualisationEngine.h>

#include <itmx/base/ITMImagePtrTypes.h>
#include <itmx/base/ITMObjectPtrTypes.h>

#include "interface/DepthVisualiser.h"
#include "interface/SemanticVisualiser.h"
#include "../util/SpaintSurfelScene.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to generate visualisations of an InfiniTAM scene.
 */
class VisualisationGenerator
{
  //#################### TYPEDEFS ####################
public:
  typedef boost::function<void(const ITMUChar4Image_CPtr&,const ITMUChar4Image_Ptr&)> Postprocessor;
  typedef boost::shared_ptr<const ITMLib::ITMSurfelVisualisationEngine<SpaintSurfel> > SurfelVisualisationEngine_CPtr;
  typedef boost::shared_ptr<const ITMLib::ITMVisualisationEngine<spaint::SpaintVoxel,ITMVoxelIndex> > VoxelVisualisationEngine_CPtr;

  //#################### ENUMERATIONS ####################
public:
  /**
   * \brief An enumeration specifying the different types of visualisation that are supported.
   */
  enum VisualisationType
  {
    VT_INPUT_COLOUR,
    VT_INPUT_DEPTH,
    VT_SCENE_COLOUR,
    VT_SCENE_CONFIDENCE,
    VT_SCENE_DEPTH,
    VT_SCENE_LAMBERTIAN,
    VT_SCENE_NORMAL,
    VT_SCENE_PHONG,
    VT_SCENE_SEMANTICCOLOUR,
    VT_SCENE_SEMANTICFLAT,
    VT_SCENE_SEMANTICLAMBERTIAN,
    VT_SCENE_SEMANTICPHONG
  };

  //#################### PRIVATE VARIABLES ####################
private:
  /** The depth visualiser. */
  DepthVisualiser_CPtr m_depthVisualiser;

  /** The label manager to use (only needed if we want to generate semantic visualisations). */
  LabelManager_CPtr m_labelManager;

  /** The semantic visualiser (only needed if we want to generate semantic visualisations). */
  SemanticVisualiser_CPtr m_semanticVisualiser;

  /** The settings to use for InfiniTAM. */
  Settings_CPtr m_settings;

  /** The InfiniTAM engine to use for rendering a surfel scene. */
  SurfelVisualisationEngine_CPtr m_surfelVisualisationEngine;

  /** The InfiniTAM engine to use for rendering a voxel scene. */
  VoxelVisualisationEngine_CPtr m_voxelVisualisationEngine;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a visualisation generator.
   *
   * \param settings                  The settings to use for InfiniTAM.
   * \param labelManager              The label manager to use (only needed if we want to generate semantic visualisations).
   * \param voxelVisualisationEngine  The InfiniTAM engine to use for rendering a voxel scene (will be created internally if NULL).
   * \param surfelVisualisationEngine The InfiniTAM engine used for rendering a surfel scene (will be created internally if NULL).
   */
  VisualisationGenerator(const Settings_CPtr& settings, const spaint::LabelManager_CPtr& labelManager = spaint::LabelManager_CPtr(),
                         const VoxelVisualisationEngine_CPtr& voxelVisualisationEngine = VoxelVisualisationEngine_CPtr(),
                         const SurfelVisualisationEngine_CPtr& surfelVisualisationEngine = SurfelVisualisationEngine_CPtr());

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Generates a synthetic depth image of a voxel scene from the specified pose.
   *
   * \note  This produces a floating-point depth image whose format matches that used by InfiniTAM,
   *        as opposed to a colourised depth image that is suitable for showing to the user.
   *
   * \param output      The location into which to put the output image.
   * \param scene       The scene to visualise.
   * \param pose        The pose from which to visualise the scene.
   * \param view        The current view of the scene (used only for the camera settings).
   * \param renderState The render state to use for intermediate storage (can be null, in which case a new one will be created).
   * \param depthType   The type of depth calculation to use.
   */
  void generate_depth_from_voxels(const ITMFloatImage_Ptr& output, const SpaintVoxelScene_CPtr& scene, const ORUtils::SE3Pose& pose,
                                  const View_CPtr& view, VoxelRenderState_Ptr& renderState, DepthVisualiser::DepthType depthType) const;

  /**
   * \brief Generates a visualisation of a surfel scene from the specified pose.
   *
   * \param output            The location into which to put the output image.
   * \param scene             The scene to visualise.
   * \param pose              The pose from which to visualise the scene.
   * \param view              The current view of the scene (used only for the camera settings).
   * \param renderState       The render state to use for intermediate storage (can be null, in which case a new one will be created).
   * \param visualisationType The type of visualisation to generate.
   */
  void generate_surfel_visualisation(const ITMUChar4Image_Ptr& output, const SpaintSurfelScene_CPtr& scene, const ORUtils::SE3Pose& pose,
                                     const View_CPtr& view, SurfelRenderState_Ptr& renderState, VisualisationType visualisationType) const;

  /**
   * \brief Generates a visualisation of a voxel scene from the specified pose.
   *
   * \param output            The location into which to put the output image.
   * \param scene             The scene to visualise.
   * \param pose              The pose from which to visualise the scene.
   * \param view              The current view of the scene (used only for the camera settings).
   * \param renderState       The render state to use for intermediate storage (can be null, in which case a new one will be created).
   * \param visualisationType The type of visualisation to generate.
   * \param postprocessor     An optional function with which to postprocess the visualisation before returning it.
   *
   * \throws std::runtime_error If supports_semantics() is false and we try to generate a semantic visualisation of the scene.
   */
  void generate_voxel_visualisation(const ITMUChar4Image_Ptr& output, const SpaintVoxelScene_CPtr& scene, const ORUtils::SE3Pose& pose,
                                    const View_CPtr& view, VoxelRenderState_Ptr& renderState, VisualisationType visualisationType,
                                    const boost::optional<Postprocessor>& postprocessor = boost::none) const;

  /**
   * \brief Gets a Lambertian raycast of a voxel scene from the default pose (the current camera pose).
   *
   * \param output          The location into which to put the output image.
   * \param liveRenderState The render state corresponding to the live camera pose for the voxel scene.
   * \param postprocessor   An optional function with which to postprocess the raycast before returning it.
   */
  void get_default_raycast(const ITMUChar4Image_Ptr& output, const VoxelRenderState_CPtr& liveRenderState, const boost::optional<Postprocessor>& postprocessor = boost::none) const;

  /**
   * \brief Gets the depth image from the most recently processed frame for a scene.
   *
   * \param output  The location into which to put the output image.
   * \param view    The current view of the scene.
   */
  void get_depth_input(const ITMUChar4Image_Ptr& output, const View_CPtr& view) const;

  /**
   * \brief Gets the RGB image from the most recently processed frame for a scene.
   *
   * \param output  The location into which to put the output image.
   * \param view    The current view of the scene.
   */
  void get_rgb_input(const ITMUChar4Image_Ptr& output, const View_CPtr& view) const;

  /**
   * \brief Gets whether or not this visualisation generator can generate semantic visualisations.
   *
   * \return  true, if this visualisation generator can generate semantic visualisations, or false otherwise.
   */
  bool supports_semantics() const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Makes a copy of an input raycast, optionally post-processes it and then ensures that it is accessible on the CPU.
   *
   * \param inputRaycast  The input raycast.
   * \param postprocessor An optional function with which to postprocess the output raycast.
   * \param outputRaycast The output raycast (guaranteed to be accessible on the CPU).
   */
  void make_postprocessed_cpu_copy(const ITMUChar4Image *inputRaycast, const boost::optional<Postprocessor>& postprocessor, const ITMUChar4Image_Ptr& outputRaycast) const;

  /**
   * \brief Prepares to copy a visualisation image into the specified output image.
   *
   * \param input   The size of the visualisation image to be copied.
   * \param output  The output image to which the visualisation image will be copied.
   */
  void prepare_to_copy_visualisation(const Vector2i& inputSize, const ITMUChar4Image_Ptr& output) const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<VisualisationGenerator> VisualisationGenerator_Ptr;
typedef boost::shared_ptr<const VisualisationGenerator> VisualisationGenerator_CPtr;

}

#endif
