/**
 * evaluation: ParamSetUtil.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "core/ParamSetUtil.h"

#include <iostream>

namespace evaluation {

//#################### STREAM OPERATORS ####################

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
