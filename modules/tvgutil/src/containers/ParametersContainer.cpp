/**
 * tvgutil: ParametersContainer.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "containers/ParametersContainer.h"

namespace tvgutil {

//#################### CONSTRUCTORS ####################
ParametersContainer::ParametersContainer() : m_container()
{
}

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
ParametersContainer& ParametersContainer::instance()
{
  static ParametersContainer container;
  return container;
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void ParametersContainer::add_value(const std::string &key, const std::string &value)
{
  m_container[key].push_back(value);
}

//#################### STREAM OPERATORS ####################
std::ostream& operator<<(std::ostream &os, const ParametersContainer &rhs)
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
