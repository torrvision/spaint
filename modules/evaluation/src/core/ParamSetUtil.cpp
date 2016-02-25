/**
 * evaluation: ParamSetUtil.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "core/ParamSetUtil.h"

#include <iostream>

namespace evaluation {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

std::string ParamSetUtil::param_set_to_string(const ParamSet& params)
{
  std::string paramString;
  size_t paramCount = params.size();
  size_t paramIndex = 0;
  for(std::map<std::string,std::string>::const_iterator it = params.begin(), iend = params.end(); it != iend; ++it)
  {
    paramString += it->first + "-" + it->second;
    if(++paramIndex != paramCount) paramString += "_";
  }
  return paramString;
}

}
