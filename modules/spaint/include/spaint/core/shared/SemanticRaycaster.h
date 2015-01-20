/**
 * spaint: SemanticRaycaster.h
 */

#ifndef H_SPAINT_SEMANTICRAYCASTER
#define H_SPAINT_SEMANTICRAYCASTER

// FIXME: Move SpaintVoxel into the shared directory.
#include "../SpaintVoxel.h"

namespace spaint {

/**
 * \brief An instance of a class deriving from this one can be used to generate a semantic raycast of an InfiniTAM scene.
 */
class SemanticRaycaster
{
  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the raycaster.
   */
  virtual ~SemanticRaycaster() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  virtual void render(const ITMScene<SpaintVoxel,ITMVoxelIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState,
                      ITMUChar4Image *output) const = 0;
};

}

#endif
