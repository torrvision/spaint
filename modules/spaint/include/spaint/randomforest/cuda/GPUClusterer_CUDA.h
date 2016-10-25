/**
 * spaint: GPUClusterer_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_GPUCLUSTERERCUDA
#define H_SPAINT_GPUCLUSTERERCUDA

#include "../interface/GPUClusterer.h"

namespace spaint
{
class GPUClusterer_CUDA: public GPUClusterer
{
public:
  GPUClusterer_CUDA(float sigma, float tau);

  virtual void find_modes(const PositionReservoir_CPtr &reservoirs,
      GPUForestPredictionsBlock_Ptr &predictions, size_t startIdx,
      size_t count);

private:
  ITMFloatImage_Ptr m_densities;
  ITMIntImage_Ptr m_parents;
  ITMIntImage_Ptr m_clusterIdx;
  ITMIntImage_Ptr m_nbClustersPerReservoir;
};
}
#endif
