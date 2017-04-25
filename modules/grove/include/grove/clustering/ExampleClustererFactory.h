/**
 * grove: ExampleClustererFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_EXAMPLECLUSTERERFACTORY
#define H_GROVE_EXAMPLECLUSTERERFACTORY

#include <ITMLib/Utils/ITMLibSettings.h>

#include "interface/ExampleClusterer.h"

namespace grove {

/**
 * \brief An instance of this struct can be used to create ExampleClusterers.
 *
 *        Clustering is performed via the "Really Quick shift" algorithm by Fulkerson and Soatto.
 *        See: http://vision.ucla.edu/~brian/papers/fulkerson10really.pdf for details.
 *
 * \note  The clusterer is capable of clustering multiple sets of examples (in parallel, when using CUDA or OpenMP),
 *        for this reason the interface to the main clustering method expects not a single set of examples but an
 *        "image" wherein each row contains a certain number examples to be clustered.
 *        Different rows are then clustered independently.
 *
 * \note  The following functions are required to be defined:
 *        - _CPU_AND_GPU_CODE_ inline float distanceSquared(const ExampleType &a, const ExampleType &b)
 *          Returns the squared distancebetween two examples.
 *        - _CPU_AND_GPU_CODE_ inline void createClusterFromExamples(const ExampleType *examples,
 *                                                                   const int *exampleKeys, int examplesCount,
 *                                                                   int key, ClusterType &outputCluster)
 *          Aggregates all the examples in the examples array having a certain key into a single cluster mode.
 *
 * \param ExampleType  The type of examples to cluster.
 * \param ClusterType  The type of clusters to generate.
 * \param MAX_CLUSTERS The maximum number of clusters to generate for each set of examples. Can be overridden with a
 *                     lesser or equal value in the make_clusterer function.
 */
template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS = 10>
class ExampleClustererFactory
{
  //#################### TYPEDEFS ####################
public:
  typedef ExampleClusterer<ExampleType, ClusterType, MAX_CLUSTERS> Clusterer;
  typedef boost::shared_ptr<Clusterer> Clusterer_Ptr;
  typedef boost::shared_ptr<const Clusterer> Clusterer_CPtr;

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
  /**
   * \brief Instantiate an ExampleClusterer.
   *
   * \param deviceType      The device used to cluster examples.
   * \param sigma           The sigma of the gaussian used to compute the example density.
   * \param tau             The maximum distance between examples to be considered part of the same example.
   * \param maxClusterCount The maximum number of clusters to extract from each example set.
   * \param minClusterSize  The minimum number of examples in a valid cluster.
   *
   * \return                An instance of ExampleClusterer.
   *
   * \throws std::invalid_argument If maxClusterCount > than MAX_CLUSTERS.
   */
  static Clusterer_Ptr make_clusterer(ITMLib::ITMLibSettings::DeviceType deviceType,
                                      float sigma,
                                      float tau,
                                      uint32_t maxClusterCount,
                                      uint32_t minClusterSize);
};

} // namespace grove

#endif // H_GROVE_EXAMPLECLUSTERERFACTORY
