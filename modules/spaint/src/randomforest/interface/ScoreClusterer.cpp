/**
 * spaint: ScoreClusterer.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "randomforest/interface/ScoreClusterer.h"

namespace spaint
{
ScoreClusterer::ScoreClusterer(float sigma, float tau, int minClusterSize) :
    m_sigma(sigma), m_tau(tau), m_minClusterSize(minClusterSize)
{
}

ScoreClusterer::~ScoreClusterer()
{
}
}
