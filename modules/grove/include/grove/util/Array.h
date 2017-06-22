/**
 * grove: Array.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_ARRAY
#define H_GROVE_ARRAY

namespace grove {

/**
 * \brief An instance of an instantiation of this struct template represents a fixed-capacity array.
 *
 * \param T         The element type.
 * \param Capacity  The array capacity.
 */
template <typename T, int Capacity>
struct Array
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
