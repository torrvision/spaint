/**
 * spaint: SpaintRaycaster.h
 */

#ifndef H_SPAINT_SPAINTRAYCASTER
#define H_SPAINT_SPAINTRAYCASTER

#include "SpaintModel.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to raycast the InfiniTAM scene in an spaint model.
 */
class SpaintRaycaster
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<ITMUChar4Image> UChar4Image_Ptr;
public:
  typedef boost::shared_ptr<ITMRenderState> RenderState_Ptr;
  typedef boost::shared_ptr<ITMVisualisationEngine<SpaintVoxel,ITMVoxelIndex> > VisualisationEngine_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The spaint model. */
  SpaintModel_CPtr m_model;

  /** The InfiniTAM engine used for raycasting the scene. */
  VisualisationEngine_Ptr m_visualisationEngine;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a raycaster.
   *
   * \param model The spaint model.
   */
  explicit SpaintRaycaster(const SpaintModel_CPtr& model);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Generates a raycast of the scene from the specified pose.
   *
   * \param output      The location into which to put the output image.
   * \param renderState The render state to use for intermediate storage.
   * \param pose        The pose from which to raycast the scene.
   */
  void generate_free_raycast(const UChar4Image_Ptr& output, RenderState_Ptr& renderState, const ITMPose& pose) const;

  /**
   * \brief Gets a raycast of the scene from the default pose (the current camera pose).
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

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Prepares to copy a visualisation image into the specified output image.
   *
   * \param input   The visualisation image to be copied.
   * \param output  The output image to which it will be copied.
   */
  template <typename T>
  void prepare_to_copy_visualisation(ORUtils::Image<T> *input, const UChar4Image_Ptr& output) const
  {
    output->Clear();
    if(m_model->get_settings().deviceType == ITMLibSettings::DEVICE_CUDA) input->UpdateHostFromDevice();
    output->ChangeDims(input->noDims);
  }
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<SpaintRaycaster> SpaintRaycaster_Ptr;
typedef boost::shared_ptr<const SpaintRaycaster> SpaintRaycaster_CPtr;

}

#endif
