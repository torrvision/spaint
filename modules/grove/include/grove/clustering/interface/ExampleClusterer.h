/**
 * grove: ExampleClusterer.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_EXAMPLECLUSTERER
#define H_GROVE_EXAMPLECLUSTERER

//#include "../base/Prediction3DColour.h"
//#include "../../keypoints/Keypoint3DColour.h"

#include <boost/shared_ptr.hpp>

#include <ORUtils/Image.h>
#include <ORUtils/MemoryBlock.h>

#include <spaint/util/ITMMemoryBlockPtrTypes.h>

namespace grove
{

template <typename ExampleType, typename ClusterType>
class ExampleClusterer
{
public:
  typedef ORUtils::MemoryBlock<ClusterType> ClusterBlock;
  typedef boost::shared_ptr<ClusterBlock> ClusterBlock_Ptr;
  typedef boost::shared_ptr<const ClusterBlock> ClusterBlock_CPtr;

  typedef ORUtils::Image<ExampleType> ExampleImage;
  typedef boost::shared_ptr<ExampleImage> ExampleImage_Ptr;
  typedef boost::shared_ptr<const ExampleImage> ExampleImage_CPtr;

public:
  ExampleClusterer(float sigma, float tau, int minClusterSize);
  virtual ~ExampleClusterer();

  virtual void find_modes(const ExampleImage_CPtr &exampleReservoirs,
                          const ITMIntMemoryBlock_CPtr &keypointReservoirsSize,
                          ClusterBlock_Ptr &predictions,
                          uint32_t startIdx, uint32_t count) = 0;

protected:
  float m_sigma;
  float m_tau;
  int m_minClusterSize;

  ITMFloatImage_Ptr m_densities;
  ITMIntImage_Ptr m_parents;
  ITMIntImage_Ptr m_clusterIdx;
  ITMIntImage_Ptr m_clusterSizes;
  ITMIntImage_Ptr m_clusterSizesHistogram;
  ITMIntImage_Ptr m_selectedClusters;
  ITMIntImage_Ptr m_nbClustersPerReservoir;
};
//
//typedef boost::shared_ptr<ExampleClusterer> ScoreClusterer_Ptr;
//typedef boost::shared_ptr<const ExampleClusterer> ScoreClusterer_CPtr;
}
#endif
