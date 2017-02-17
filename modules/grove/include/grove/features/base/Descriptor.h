/**
 * grove: Descriptor.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_DESCRIPTOR
#define H_GROVE_DESCRIPTOR

namespace grove
{

/**
 * \brief An instance of this type represents a fixed-length floating point Descriptor.
 *
 * \param LENGTH Length of the descriptor.
 */
template<uint32_t LENGTH>
struct Descriptor
{
  //#################### CONSTANTS ####################

  /** Length of the descriptor. */
  static const int FEATURE_COUNT = LENGTH;

  //#################### PUBLIC VARIABLES ####################

  /** Features are stored in this array. */
  float data[FEATURE_COUNT];
};

}

#endif
