/**
 * evaluation: ParamSet.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "core/ParamSet.h"

#include <iostream>

namespace evaluation {

//#################### STREAM OPERATORS ####################

#if 0
std::ostream& operator<<(std::ostream& os, const ParamSet& rhs)
{
  size_t size = rhs.size();
  size_t count = 0;
  for(ParamSet::const_iterator it = rhs.begin(), iend = rhs.end(); it != iend; ++it)
  {
    os << it->first << '-' << it->second;
    if(++count != size) os << '_';
  }
  return os;
}
#endif

std::string ParamSetUtil::param_set_to_string(const ParamSet& params)
{
  std::string paramString;
  size_t mapSize = params.size();
  size_t sizeCount = 0;
  for(std::map<std::string,std::string>::const_iterator it = params.begin(), iend = params.end(); it != iend; ++it)
  {
    paramString += it->first + "-" + it->second;
    if(++sizeCount != mapSize) paramString += "_";
  }

  return paramString;
}

}
