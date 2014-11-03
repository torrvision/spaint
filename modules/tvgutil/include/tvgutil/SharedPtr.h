/**
 * tvgutil: SharedPtr.h
 */

#ifndef H_TVGUTIL_SHAREDPTR
#define H_TVGUTIL_SHAREDPTR

#if WITH_BOOST

#include <boost/shared_ptr.hpp>

namespace tvgutil {
  using boost::shared_ptr;
}

#else

// TODO: Find a non-Boost solution that works for older compilers.
#include <memory>

namespace tvgutil {
  using std::shared_ptr;
}

#endif

#endif
