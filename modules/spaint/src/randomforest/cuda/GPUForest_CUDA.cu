/**
 * spaint: GPUForest_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "randomforest/cuda/GPUForest_CUDA.h"

namespace spaint
{
GPUForest_CUDA::GPUForest_CUDA(const EnsembleLearner &pretrained_forest) :
    GPUForest(pretrained_forest)
{
  m_forestImage->UpdateDeviceFromHost();
}

void GPUForest_CUDA::evaluate_forest(const RGBDPatchFeatureImage_CPtr &features,
    ITMIntImage_Ptr &leaf_indices) const
{

}
}
