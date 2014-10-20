/**
 * spaint: SharedPtr.h
 */

#ifndef H_SPAINT_SHAREDPTR
#define H_SPAINT_SHAREDPTR

#if WITH_BOOST

#include <boost/shared_ptr.hpp>

namespace spaint {
  using boost::shared_ptr;
}

#else

// TODO: Find a non-Boost solution that works for older compilers.
#include <memory>

namespace spaint {
  using std::shared_ptr;
}

#endif

#endif
