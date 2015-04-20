/**
 * spaint: SemanticVisualiser_CUDA.h
 */

#ifndef H_SPAINT_SEMANTICVISUALISER_CUDA
#define H_SPAINT_SEMANTICVISUALISER_CUDA

#include "../interface/SemanticVisualiser.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to render a semantic visualisation of an InfiniTAM scene using CUDA.
 */
class SemanticVisualiser_CUDA : public SemanticVisualiser
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a CUDA-based semantic visualiser.
   *
   * \param labelColours  The colours to use for the semantic labels.
   */
  explicit SemanticVisualiser_CUDA(const std::vector<Vector3u>& labelColours);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void render(const ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene, const ITMLib::Objects::ITMPose *pose,
                      const ITMLib::Objects::ITMIntrinsics *intrinsics, const ITMLib::Objects::ITMRenderState *renderState,
                      bool usePhong, ITMUChar4Image *outputImage) const;
};

}

#endif
