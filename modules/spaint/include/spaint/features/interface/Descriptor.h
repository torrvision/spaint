/**
 * spaint: Descriptor.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_DESCRIPTOR
#define H_SPAINT_DESCRIPTOR

#include <boost/shared_ptr.hpp>

#include <ITMLib/Utils/ITMMath.h>

#include <ORUtils/Image.h>

namespace spaint
{

/**
 * \brief An instance of this type represents a fixed-length floating point Descriptor.
 *
 * \param FEATURE_COUNT Length of the descriptor.
 */
template<uint32_t LENGTH>
struct Descriptor
{
  //#################### CONSTANTS ####################

  /** Length of the descriptor. */
  static const int FEATURE_COUNT = LENGTH;

  //#################### PUBLIC VARIABLES ####################

  /** All the features are stored in this array. */
  float data[FEATURE_COUNT];
};

//#################### TYPEDEFS ####################

typedef Descriptor<256> RGBDPatchDescriptor;

typedef ORUtils::Image<RGBDPatchDescriptor> RGBDPatchDescriptorImage;
typedef boost::shared_ptr<RGBDPatchDescriptorImage> RGBDPatchDescriptorImage_Ptr;
typedef boost::shared_ptr<const RGBDPatchDescriptorImage> RGBDPatchDescriptorImage_CPtr;

}

#endif
