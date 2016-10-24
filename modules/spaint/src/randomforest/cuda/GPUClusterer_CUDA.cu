/**
 * spaint: GPUClusterer_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "randomforest/cuda/GPUClusterer_CUDA.h"

namespace spaint
{
GPUClusterer_CUDA::GPUClusterer_CUDA(float sigma, float tau) :
    GPUClusterer(sigma, tau)
{
}

void GPUClusterer_CUDA::find_modes(const PositionReservoir_CPtr &reservoirs,
    GPUForestPredictionsBlock_Ptr &predictions, size_t startIdx, size_t count)
{

}

}
