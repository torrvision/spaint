/**
 * spaint: SemanticVisualiser.h
 */

#ifndef H_SPAINT_SEMANTICVISUALISER
#define H_SPAINT_SEMANTICVISUALISER

#include <ITMLib/Objects/ITMIntrinsics.h>
#include <ITMLib/Objects/ITMPose.h>
#include <ITMLib/Objects/ITMRenderState.h>
#include <ITMLib/Objects/ITMScene.h>

#include "../../../util/SpaintVoxel.h"

namespace spaint {

/**
 * \brief An instance of a class deriving from this one can be used to render a semantic visualisation of an InfiniTAM scene.
 *
 * By "semantic visualisation", we mean an image showing the voxels of the scene labelled with various different classes, e.g. floor, table, etc.
 */
class SemanticVisualiser
{
  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the semantic visualiser.
   */
  virtual ~SemanticVisualiser() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Renders a semantic view of the specified scene from the specified camera pose.
   *
   * \param scene       The scene.
   * \param pose        The camera pose.
   * \param intrinsics  The intrinsic parameters of the camera.
   * \param renderState The render state corresponding to the specified camera pose.
   * \param outputImage The image into which to write the semantic visualisation of the scene.
   */
  virtual void render(const ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene, const ITMLib::Objects::ITMPose *pose,
                      const ITMLib::Objects::ITMIntrinsics *intrinsics, const ITMLib::Objects::ITMRenderState *renderState,
                      ITMUChar4Image *outputImage) const = 0;
};

}

#endif
