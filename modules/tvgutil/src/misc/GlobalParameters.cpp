/**
 * tvgutil: GlobalParameters.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "misc/GlobalParameters.h"

namespace tvgutil {

//#################### SINGLETON IMPLEMENTATION ####################

GlobalParameters::GlobalParameters() {}

GlobalParameters& GlobalParameters::instance()
{
  static GlobalParameters s_instance;
  return s_instance;
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void GlobalParameters::add_value(const std::string& key, const std::string& value)
{
  m_params[key].push_back(value);
}

//#################### STREAM OPERATORS ####################

std::ostream& operator<<(std::ostream& os, const GlobalParameters& rhs)
{
  for(std::map<std::string,std::vector<std::string> >::const_iterator it = rhs.m_params.begin(), iend = rhs.m_params.end(); it != iend; ++it)
  {
    os << it->first << ":\n";
    for(std::vector<std::string>::const_iterator jt = it->second.begin(), jend = it->second.end(); jt != jend; ++jt)
    {
      os << '\t' << *jt << '\n';
    }
  }

  return os;
}

}
