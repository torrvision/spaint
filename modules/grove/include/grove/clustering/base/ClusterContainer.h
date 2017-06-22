/**
 * grove: ClusterContainer.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_CLUSTERCONTAINER
#define H_GROVE_CLUSTERCONTAINER

namespace grove {

/**
 * \brief An instance of an instantiation of this struct template represents a fixed-capacity container
 *        of typed clusters (e.g. as output by an example clusterer).
 *
 * \param T         The type of the contained clusters.
 * \param Capacity  The maximum number of clusters that can be stored in the container.
 */
template <typename T, int Capacity>
struct ClusterContainer
{
  //#################### TYPEDEFS ####################

  typedef T ClusterType;

  //#################### ENUMERATIONS ####################

  /** The maximum number of modal clusters associated to the prediction. */
  enum { MAX_CLUSTERS = Capacity };

  //#################### PUBLIC MEMBER VARIABLES ####################

  /** The clusters. */
  T clusters[MAX_CLUSTERS];

  /** The number of clusters currently stored in the container. */
  int size;
};

}

#endif
