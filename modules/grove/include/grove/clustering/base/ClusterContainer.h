/**
 * grove: ClusterContainer.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_CLUSTERCONTAINER
#define H_GROVE_CLUSTERCONTAINER

namespace grove {

/**
 * \brief An instance of this struct represents a fixed-size container of typed clusters output
 *        by the ExampleClusterer class.
 *
 * \param T    The type of the contained clusters.
 * \param SIZE The maximum number of clusters contained in a ClusterContainer.
 */
template <typename T, int SIZE = 10>
struct ClusterContainer
{
  //#################### TYPEDEFS ####################
  typedef T ClusterType;

  //#################### ENUMS ####################
  /** The maximum number of modal clusters associated to the prediction. */
  enum { MAX_CLUSTERS = SIZE };

  //#################### MEMBER VARIABLES ####################
  /** The modal clusters. */
  T clusters[MAX_CLUSTERS];

  /** The actual number of modes stored in the struct. */
  int nbClusters;
};

} // namespace grove

#endif // H_GROVE_CLUSTERCONTAINER
