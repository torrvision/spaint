/**
 * rafl: Descriptor.h
 */

#ifndef H_RAFL_DESCRIPTOR
#define H_RAFL_DESCRIPTOR

#include <vector>
#include <tvgutil/SharedPtr.h>

namespace rafl {

typedef std::vector<float> Descriptor;
typedef tvgutil::shared_ptr<Descriptor> Descriptor_Ptr;
typedef tvgutil::shared_ptr<const Descriptor> Descriptor_CPtr;

}

#endif
