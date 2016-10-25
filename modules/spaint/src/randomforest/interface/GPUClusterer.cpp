/**
 * spaint: GPUClusterer.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "randomforest/interface/GPUClusterer.h"

namespace spaint
{
GPUClusterer::GPUClusterer(float sigma, float tau, int minClusterSize) :
    m_sigma(sigma), m_tau(tau), m_minClusterSize(minClusterSize)
{
}

GPUClusterer::~GPUClusterer()
{
}
}
