/**
 * spaint: SemanticRaycastImpl.h
 */

#ifndef H_SPAINT_SEMANTICRAYCASTIMPL
#define H_SPAINT_SEMANTICRAYCASTIMPL

#include <ITMLib/Objects/ITMIntrinsics.h>
#include <ITMLib/Objects/ITMPose.h>
#include <ITMLib/Objects/ITMRenderState.h>
#include <ITMLib/Objects/ITMScene.h>

#include "../../voxels/SpaintVoxel.h"

namespace spaint {

/**
 * \brief An instance of a class deriving from this one can be used to generate a semantic raycast of an InfiniTAM scene.
 */
class SemanticRaycastImpl
{
  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the semantic raycast implementation.
   */
  virtual ~SemanticRaycastImpl() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  virtual void render(const ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene, const ITMLib::Objects::ITMPose *pose,
                      const ITMLib::Objects::ITMIntrinsics *intrinsics, const ITMLib::Objects::ITMRenderState *renderState,
                      ITMUChar4Image *outputImage) const = 0;
};

}

#endif
