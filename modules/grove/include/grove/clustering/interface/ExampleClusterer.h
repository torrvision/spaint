/**
 * grove: ExampleClusterer.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_EXAMPLECLUSTERER
#define H_GROVE_EXAMPLECLUSTERER

#include <boost/shared_ptr.hpp>

#include <ORUtils/Image.h>
#include <ORUtils/MemoryBlock.h>

#include <itmx/ITMImagePtrTypes.h>
#include <itmx/ITMMemoryBlockPtrTypes.h>

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
  ExampleClusterer(float sigma, float tau, uint32_t maxClusterCount, uint32_t minClusterSize);
  virtual ~ExampleClusterer();

  void find_modes(const ExampleImage_CPtr &exampleReservoirs,
                  const ITMIntMemoryBlock_CPtr &keypointReservoirsSize,
                  ClusterBlock_Ptr &predictions,
                  uint32_t startIdx, uint32_t count);

protected:
  float m_sigma;
  float m_tau;
  uint32_t m_maxClusterCount;
  uint32_t m_minClusterSize;

  ITMFloatImage_Ptr m_densities;
  ITMIntImage_Ptr m_parents;
  ITMIntImage_Ptr m_clusterIdx;
  ITMIntImage_Ptr m_clusterSizes;
  ITMIntImage_Ptr m_clusterSizesHistogram;
  ITMIntImage_Ptr m_selectedClusters;
  ITMIntMemoryBlock_Ptr m_nbClustersPerReservoir;

protected:
  virtual const ExampleType* get_pointer_to_reservoir(const ExampleImage_CPtr &exampleReservoirs, uint32_t reservoirIdx) const = 0;
  virtual const int* get_pointer_to_reservoir_size(const ITMIntMemoryBlock_CPtr &exampleReservoirsSize, uint32_t reservoirIdx) const = 0;
  virtual ClusterType* get_pointer_to_prediction(const ClusterBlock_Ptr &predictions, uint32_t predictionIdx) const = 0;

  virtual void reset_temporaries(uint32_t reservoirCapacity, uint32_t reservoirCount) = 0;
  virtual void reset_predictions(ClusterType *predictionsData, uint32_t reservoirCount) const = 0;
  virtual void compute_density(const ExampleType *examples, const int *reservoirSizes, uint32_t reservoirCapacity, uint32_t reservoirCount, float sigma) = 0;
  virtual void link_neighbors(const ExampleType *examples, const int *reservoirSizes, uint32_t reservoirCapacity, uint32_t reservoirCount, float tauSq) = 0;
  virtual void identify_clusters(uint32_t reservoirCapacity, uint32_t reservoirCount) = 0;
  virtual void compute_cluster_size_histograms(uint32_t reservoirCapacity, uint32_t reservoirCount) = 0;
  virtual void select_clusters(uint32_t maxClusterCount, uint32_t minClusterSize, uint32_t reservoirCapacity, uint32_t reservoirCount) = 0;
  virtual void compute_modes(const ExampleType *examples, const int *reservoirSizes, ClusterType *predictionsData, uint32_t maxClusterCount, uint32_t reservoirCapacity, uint32_t reservoirCount) = 0;

private:
  void allocate_temporaries(uint32_t reservoirCapacity, uint32_t reservoirCount);
};

}
#endif
