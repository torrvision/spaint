/**
 * spaint: SemanticVisualiser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_SEMANTICVISUALISER
#define H_SPAINT_SEMANTICVISUALISER

#include <ITMLib/Objects/Camera/ITMIntrinsics.h>
#include <ITMLib/Objects/RenderStates/ITMRenderState.h>
#include <ITMLib/Utils/ITMImageTypes.h>

#include <ORUtils/SE3Pose.h>

#include "../shared/SemanticVisualiser_Settings.h"
#include "../../util/LabelManager.h"
#include "../../util/SpaintScene.h"

namespace spaint {

/**
 * \brief An instance of a class deriving from this one can be used to render a semantic visualisation of an InfiniTAM scene.
 *
 * By "semantic visualisation", we mean an image showing the voxels of the scene labelled with various different classes, e.g. floor, table, etc.
 */
class SemanticVisualiser
{
  //#################### PROTECTED VARIABLES ####################
protected:
  /** A memory block in which to store the colours to use for the semantic labels. */
  boost::shared_ptr<ORUtils::MemoryBlock<Vector3u> > m_labelColoursMB;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a semantic visualiser.
   *
   * \param maxLabelCount The maximum number of labels that can be in use.
   */
  explicit SemanticVisualiser(size_t maxLabelCount);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the semantic visualiser.
   */
  virtual ~SemanticVisualiser();

  //#################### PRIVATE ABSTRACT MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Renders a semantic view of the specified scene from the specified camera pose.
   *
   * \param scene         The scene.
   * \param pose          The camera pose.
   * \param intrinsics    The intrinsic parameters of the camera.
   * \param renderState   The render state corresponding to the specified camera pose.
   * \param lightingType  The type of lighting to use.
   * \param labelAlpha    The proportion (in the range [0,1]) of the final pixel colours that should be based on the voxels' semantic labels rather than their scene colours.
   * \param outputImage   The image into which to write the semantic visualisation of the scene.
   */
  virtual void render_internal(const Scene *scene, const ORUtils::SE3Pose *pose, const ITMLib::ITMIntrinsics *intrinsics, const ITMLib::ITMRenderState *renderState,
                               LightingType lightingType, float labelAlpha, ITMUChar4Image *outputImage) const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Renders a semantic view of the specified scene from the specified camera pose.
   *
   * \param scene         The scene.
   * \param pose          The camera pose.
   * \param intrinsics    The intrinsic parameters of the camera.
   * \param renderState   The render state corresponding to the specified camera pose.
   * \param labelColours  The colours to use for the semantic labels.
   * \param lightingType  The type of lighting to use.
   * \param labelAlpha    The proportion (in the range [0,1]) of the final pixel colours that should be based on the voxels' semantic labels rather than their scene colours.
   * \param outputImage   The image into which to write the semantic visualisation of the scene.
   */
  void render(const Scene *scene, const ORUtils::SE3Pose *pose, const ITMLib::ITMIntrinsics *intrinsics, const ITMLib::ITMRenderState *renderState,
              const std::vector<Vector3u>& labelColours, LightingType lightingType, float labelAlpha, ITMUChar4Image *outputImage) const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<const SemanticVisualiser> SemanticVisualiser_CPtr;

}

#endif
