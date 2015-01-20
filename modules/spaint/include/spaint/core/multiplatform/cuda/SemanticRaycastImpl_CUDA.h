/**
 * spaint: SemanticRaycastImpl_CUDA.h
 */

#ifndef H_SPAINT_SEMANTICRAYCASTIMPL_CUDA
#define H_SPAINT_SEMANTICRAYCASTIMPL_CUDA

#include "../interface/SemanticRaycastImpl.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to generate a semantic raycast of an InfiniTAM scene using CUDA.
 */
class SemanticRaycastImpl_CUDA : public SemanticRaycastImpl
{
  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void render(const ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene, const ITMLib::Objects::ITMPose *pose,
                      const ITMLib::Objects::ITMIntrinsics *intrinsics, const ITMLib::Objects::ITMRenderState *renderState,
                      ITMUChar4Image *outputImage) const;
};

}

#endif
