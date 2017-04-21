/**
 * tvgutil: GlobalParameters.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "misc/GlobalParameters.h"

namespace tvgutil {

//#################### CONSTRUCTORS ####################

GlobalParameters::GlobalParameters() {}

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

GlobalParameters& GlobalParameters::instance()
{
  static GlobalParameters container;
  return container;
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void GlobalParameters::add_value(const std::string &key, const std::string &value)
{
  m_container[key].push_back(value);
}

//#################### STREAM OPERATORS ####################

std::ostream& operator<<(std::ostream &os, const GlobalParameters& rhs)
{
  for(std::map<std::string, std::vector<std::string> >::const_iterator it = rhs.m_container.begin(); it != rhs.m_container.end(); ++it)
  {
    os << it->first << ":\n";
    for(std::vector<std::string>::const_iterator valueIt = it->second.begin(); valueIt != it->second.end(); ++valueIt)
    {
      os << '\t' << *valueIt << '\n';
    }
  }
  return os;
}

}
