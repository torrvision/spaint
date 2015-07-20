/**
 * spaint: LabelPropagator_CPU.h
 */

#ifndef H_SPAINT_LABELPROPAGATOR_CPU
#define H_SPAINT_LABELPROPAGATOR_CPU

#include "../interface/LabelPropagator.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to propagate a specified label across surfaces in the scene using the CPU.
 */
class LabelPropagator_CPU : public LabelPropagator
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a CPU-based label propagator.
   *
   * \param raycastResultSize The size of the raycast result (in pixels).
   */
  explicit LabelPropagator_CPU(size_t raycastResultSize);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /** Override */
  virtual void calculate_normals(const ITMFloat4Image *raycastResult, const ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene) const;

  /** Override */
  virtual void perform_propagation(SpaintVoxel::Label label, const ITMFloat4Image *raycastResult,
                                   ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene) const;
};

}

#endif
