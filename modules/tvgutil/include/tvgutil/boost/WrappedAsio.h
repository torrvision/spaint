/**
 * tvgutil: WrappedAsio.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifdef _MSC_VER
  // Suppress some VC++ warnings that are produced by boost/asio.hpp.
  #pragma warning(disable:4267 4996)
#endif

#include <boost/asio.hpp>

#ifdef _MSC_VER
  // Re-enable the VC++ warnings for the rest of the code.
  #pragma warning(default:4267 4996)
#endif
