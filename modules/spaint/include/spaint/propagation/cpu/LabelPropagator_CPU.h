/**
 * spaint: LabelPropagator_CPU.h
 */

#ifndef H_SPAINT_LABELPROPAGATOR_CPU
#define H_SPAINT_LABELPROPAGATOR_CPU

#include "../interface/LabelPropagator.h"

namespace spaint {

/**
 * \brief TODO
 */
class LabelPropagator_CPU : public LabelPropagator
{
  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /** Override */
  virtual void calculate_normals(const ITMFloat4Image *raycastResult, const ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene) const;
};

}

#endif
