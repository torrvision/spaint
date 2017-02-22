/**
 * spaint: ScoreClusterer.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SCORECLUSTERER
#define H_SPAINT_SCORECLUSTERER

#include "../ScoreForestTypes.h"

#include <boost/shared_ptr.hpp>

#include <grove/keypoints/Keypoint3DColour.h>
#include <grove/reservoirs/interface/ExampleReservoirs.h>
using namespace grove;

namespace spaint
{
typedef ExampleReservoirs<Keypoint3DColour, LeafIndices> PositionReservoir;
typedef boost::shared_ptr<PositionReservoir> PositionReservoir_Ptr;
typedef boost::shared_ptr<const PositionReservoir> PositionReservoir_CPtr;

class ScoreClusterer
{
public:
  ScoreClusterer(float sigma, float tau, int minClusterSize);
  virtual ~ScoreClusterer();

  virtual void find_modes(const PositionReservoir_CPtr &reservoirs,
      ScorePredictionsBlock_Ptr &predictions, size_t startIdx,
      size_t count) = 0;

protected:
  float m_sigma;
  float m_tau;
  int m_minClusterSize;
};

typedef boost::shared_ptr<ScoreClusterer> ScoreClusterer_Ptr;
typedef boost::shared_ptr<const ScoreClusterer> ScoreClusterer_CPtr;
}
#endif
