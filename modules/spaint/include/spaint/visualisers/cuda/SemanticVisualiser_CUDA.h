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
   * \param maxLabelCount The maximum number of labels that can be in use.
   */
  explicit SemanticVisualiser_CUDA(size_t maxLabelCount);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /** Override */
  virtual void render_internal(const ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene, const ITMLib::Objects::ITMPose *pose,
                               const ITMLib::Objects::ITMIntrinsics *intrinsics, const ITMLib::Objects::ITMRenderState *renderState,
                               bool usePhong, float labelAlpha, ITMUChar4Image *outputImage) const;
};

}

#endif
