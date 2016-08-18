/**
 * spaintgui: Raycaster.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINTGUI_RAYCASTER
#define H_SPAINTGUI_RAYCASTER

#include <boost/function.hpp>
#include <boost/optional.hpp>

#include <ITMLib/Engines/Visualisation/Interface/ITMVisualisationEngine.h>

#include <spaint/util/ITMImagePtrTypes.h>
#include <spaint/visualisers/interface/SemanticVisualiser.h>

#include "Model.h"

/**
 * \brief An instance of this class can be used to raycast the InfiniTAM scene in an spaint model.
 */
class Raycaster
{
  //#################### TYPEDEFS ####################
public:
  typedef boost::function<void(const ITMUChar4Image_CPtr&,const ITMUChar4Image_Ptr&)> Postprocessor;
  typedef boost::shared_ptr<ITMLib::ITMRenderState> RenderState_Ptr;
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;
  typedef boost::shared_ptr<const ITMLib::ITMLibSettings> Settings_CPtr;
  typedef boost::shared_ptr<ITMLib::ITMVisualisationEngine<spaint::SpaintVoxel,ITMVoxelIndex> > VisualisationEngine_Ptr;

  //#################### ENUMERATIONS ####################
public:
  /**
   * \brief An enumeration specifying the different types of free-view raycasting that are supported.
   */
  enum RaycastType
  {
    RT_COLOUR,
    RT_LAMBERTIAN,
    RT_SEMANTICCOLOUR,
    RT_SEMANTICFLAT,
    RT_SEMANTICLAMBERTIAN,
    RT_SEMANTICPHONG
  };

  //#################### PRIVATE VARIABLES ####################
private:
  /** The spaint model. */
  Model_CPtr m_model;

  /** The platform-specific semantic visualiser. */
  boost::shared_ptr<const spaint::SemanticVisualiser> m_semanticVisualiser;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a raycaster.
   *
   * \param model             The spaint model.
   * \param trackedImageSize  The size of the images used by the tracker.
   * \param settings          The settings to use for InfiniTAM.
   */
  Raycaster(const Model_CPtr& model, const Vector2i& trackedImageSize, const Settings_CPtr& settings);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Generates a raycast of the scene from the specified pose.
   *
   * \param output        The location into which to put the output image.
   * \param renderState   The render state to use for intermediate storage.
   * \param pose          The pose from which to raycast the scene.
   * \param raycastType   The type of raycast to generate.
   * \param postprocessor An optional function with which to postprocess the raycast before returning it.
   */
  void generate_free_raycast(const ITMUChar4Image_Ptr& output, RenderState_Ptr& renderState, const ORUtils::SE3Pose& pose, RaycastType = RT_LAMBERTIAN,
                             const boost::optional<Postprocessor>& postprocessor = boost::none) const;

  /**
   * \brief Gets a Lambertian raycast of the scene from the default pose (the current camera pose).
   *
   * \param liveRenderState The render state corresponding to the current camera pose.
   * \param output          The location into which to put the output image.
   * \param postprocessor   An optional function with which to postprocess the raycast before returning it.
   */
  void get_default_raycast(const RenderState_CPtr& liveRenderState, const ITMUChar4Image_Ptr& output, const boost::optional<Postprocessor>& postprocessor = boost::none) const;

  /**
   * \brief Gets the depth image from the most recently processed frame.
   *
   * \param output  The location into which to put the output image.
   */
  void get_depth_input(const ITMUChar4Image_Ptr& output) const;

  /**
   * \brief Gets the RGB image from the most recently processed frame.
   *
   * \param output  The location into which to put the output image.
   */
  void get_rgb_input(const ITMUChar4Image_Ptr& output) const;

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

typedef boost::shared_ptr<Raycaster> Raycaster_Ptr;
typedef boost::shared_ptr<const Raycaster> Raycaster_CPtr;

#endif
