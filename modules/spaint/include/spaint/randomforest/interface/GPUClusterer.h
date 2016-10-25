/**
 * spaint: GPUClusterer.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_GPUCLUSTERER
#define H_SPAINT_GPUCLUSTERER

#include "GPUReservoir.h"
#include "GPUForestTypes.h"

#include <boost/shared_ptr.hpp>

namespace spaint
{
class GPUClusterer
{
public:
  GPUClusterer(float sigma, float tau, int minClusterSize);
  virtual ~GPUClusterer();

  virtual void find_modes(const PositionReservoir_CPtr &reservoirs,
      GPUForestPredictionsBlock_Ptr &predictions, size_t startIdx,
      size_t count) = 0;

protected:
  float m_sigma;
  float m_tau;
  int m_minClusterSize;
};

typedef boost::shared_ptr<GPUClusterer> GPUClusterer_Ptr;
typedef boost::shared_ptr<const GPUClusterer> GPUClusterer_CPtr;
}
#endif
