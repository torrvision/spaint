/**
 * tvgutil: SettingsContainer.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "misc/SettingsContainer.h"

namespace tvgutil {

//#################### SINGLETON IMPLEMENTATION ####################

SettingsContainer::SettingsContainer() {}

SettingsContainer& SettingsContainer::instance()
{
  static SettingsContainer s_instance;
  return s_instance;
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void SettingsContainer::add_value(const std::string& key, const std::string& value)
{
  m_params[key].push_back(value);
}

//#################### STREAM OPERATORS ####################

std::ostream& operator<<(std::ostream& os, const SettingsContainer& rhs)
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
