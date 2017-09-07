/**
 * tvgutil: ConversionUtil.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_TVGUTIL_CONVERSIONUTIL
#define H_TVGUTIL_CONVERSIONUTIL

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

namespace tvgutil {

/**
 * \brief Converts a string to the specified type.
 *
 * \param in  The string to convert.
 * \return    The result of the conversion.
 */
template <typename T>
inline T from_string(const std::string& in)
{
  return boost::lexical_cast<T>(in);
}

/**
 * \brief Converts a string to a bool.
 *
 * \param in  The string to convert.
 * \return    The result of the conversion.
 */
template <>
inline bool from_string(const std::string& in)
{
  std::string copy = boost::to_lower_copy(in);

  if(copy == "true") return true;
  else if(copy == "false") return false;
  else return boost::lexical_cast<bool>(in);
}

}

#endif
