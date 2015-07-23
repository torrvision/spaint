/**
 * rafl: Descriptor.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_RAFL_DESCRIPTOR
#define H_RAFL_DESCRIPTOR

#include <vector>

#include <boost/shared_ptr.hpp>

namespace rafl {

typedef std::vector<float> Descriptor;
typedef boost::shared_ptr<Descriptor> Descriptor_Ptr;
typedef boost::shared_ptr<const Descriptor> Descriptor_CPtr;

}

#endif
