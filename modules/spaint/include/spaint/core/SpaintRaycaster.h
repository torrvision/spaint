/**
 * spaint: SpaintRaycaster.h
 */

#ifndef H_SPAINT_SPAINTRAYCASTER
#define H_SPAINT_SPAINTRAYCASTER

#include <boost/optional.hpp>

#include "SpaintModel.h"
#include "../visualisers/multiplatform/interface/SemanticVisualiser.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to raycast the InfiniTAM scene in an spaint model.
 */
class SpaintRaycaster
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<ITMFloat4Image> Float4Image_Ptr;
  typedef boost::shared_ptr<ITMUChar4Image> UChar4Image_Ptr;
public:
  typedef boost::shared_ptr<ITMRenderState> RenderState_Ptr;
  typedef boost::shared_ptr<const ITMRenderState> RenderState_CPtr;
  typedef boost::shared_ptr<ITMVisualisationEngine<SpaintVoxel,ITMVoxelIndex> > VisualisationEngine_Ptr;

  //#################### ENUMERATIONS ####################
public:
  /**
   * \brief An enumeration specifying the different types of free-view raycasting that are supported.
   */
  enum RaycastType
  {
    RT_COLOUR,
    RT_LAMBERTIAN,
    RT_SEMANTIC
  };

  //#################### PRIVATE VARIABLES ####################
private:
  /** The render state corresponding to the live camera pose. */
  RenderState_Ptr m_liveRenderState;

  /** The spaint model. */
  SpaintModel_CPtr m_model;

  /** An image into which the raycast result may be copied when performing picking. */
  mutable Float4Image_Ptr m_raycastResult;

  /** The platform-specific semantic visualiser. */
  boost::shared_ptr<const SemanticVisualiser> m_semanticVisualiser;

  /** The InfiniTAM engine used for raycasting the scene. */
  VisualisationEngine_Ptr m_visualisationEngine;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a raycaster.
   *
   * \param model               The spaint model.
   * \param visualisationEngine The InfiniTAM engine used for raycasting the scene.
   * \param liveRenderState     The render state corresponding to the live camera pose.
   */
  SpaintRaycaster(const SpaintModel_CPtr& model, const VisualisationEngine_Ptr& visualisationEngine, const RenderState_Ptr& liveRenderState);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Generates a raycast of the scene from the specified pose.
   *
   * \param output      The location into which to put the output image.
   * \param renderState The render state to use for intermediate storage.
   * \param pose        The pose from which to raycast the scene.
   * \param raycastType The type of raycast to generate.
   */
  void generate_free_raycast(const UChar4Image_Ptr& output, RenderState_Ptr& renderState, const ITMPose& pose, RaycastType = RT_LAMBERTIAN) const;

  /**
   * \brief Gets a Lambertian raycast of the scene from the default pose (the current camera pose).
   *
   * \param output  The location into which to put the output image.
   */
  void get_default_raycast(const UChar4Image_Ptr& output) const;

  /**
   * \brief Gets the depth image from the most recently processed frame.
   *
   * \param output  The location into which to put the output image.
   */
  void get_depth_input(const UChar4Image_Ptr& output) const;

  /**
   * \brief Gets the render state corresponding to the live camera pose.
   *
   * \return  The render state corresponding to the live camera pose.
   */
  const RenderState_Ptr& get_live_render_state();

  /**
   * \brief Gets the RGB image from the most recently processed frame.
   *
   * \param output  The location into which to put the output image.
   */
  void get_rgb_input(const UChar4Image_Ptr& output) const;

  /**
   * \brief Gets the InfiniTAM engine used for raycasting the scene.
   *
   * \return  The InfiniTAM engine used for raycasting the scene.
   */
  const VisualisationEngine_Ptr& get_visualisation_engine();

  /**
   * \brief Determines the nearest scene point (if any) that would be hit by a ray cast through (x,y) on the image plane
   *        when viewed from the camera pose with the specified render state.
   *
   * \param x           The x coordinate of the point on the image plane through which the ray is cast.
   * \param y           The y coordinate of the point on the image plane through which the ray is cast.
   * \param renderState An optional render state corresponding to a particular camera pose (if null, the live render state is used).
   * \return            The coordinates of the nearest scene point (if any) that is hit by the ray.
   */
  boost::optional<Vector3f> pick(int x, int y, RenderState_CPtr renderState = RenderState_CPtr()) const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Prepares to copy a visualisation image into the specified output image.
   *
   * \param input   The size of the visualisation image to be copied.
   * \param output  The output image to which the visualisation image will be copied.
   */
  void prepare_to_copy_visualisation(const Vector2i& inputSize, const UChar4Image_Ptr& output) const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<SpaintRaycaster> SpaintRaycaster_Ptr;
typedef boost::shared_ptr<const SpaintRaycaster> SpaintRaycaster_CPtr;

}

#endif
