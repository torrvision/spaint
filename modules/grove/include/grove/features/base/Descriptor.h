/**
 * grove: Descriptor.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_DESCRIPTOR
#define H_GROVE_DESCRIPTOR

namespace grove {

/**
 * \brief An instance of an instantiation of this struct template represents a fixed-length, floating-point feature descriptor.
 *
 * \tparam N The length of the descriptor.
 */
template <int N>
struct Descriptor
{
  //#################### CONSTANTS ####################

  /** The length of the descriptor. */
  static const int FEATURE_COUNT = N;

  //#################### PUBLIC VARIABLES ####################

  /** The descriptor's features are stored in this array. */
  float data[FEATURE_COUNT];
};

}

#endif
