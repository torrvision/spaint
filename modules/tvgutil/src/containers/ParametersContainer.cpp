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

bool ParametersContainer::get_bool(const std::string &key) const
{
  return get_typed_value<bool>(key);
}

bool ParametersContainer::get_bool(const std::string &key, bool defaultValue) const
{
  return get_typed_value<bool>(key, defaultValue);
}

int ParametersContainer::get_int(const std::string &key) const
{
  return get_typed_value<int>(key);
}

int ParametersContainer::get_int(const std::string &key, int defaultValue) const
{
  return get_typed_value<int>(key, defaultValue);
}

float ParametersContainer::get_float(const std::string &key) const
{
  return get_typed_value<float>(key);
}

float ParametersContainer::get_float(const std::string &key, float defaultValue) const
{
  return get_typed_value<float>(key, defaultValue);
}

std::vector<std::string> ParametersContainer::get_multiple_strings(const std::string &key) const
{
  return MapUtil::lookup(m_container, key);
}

std::vector<std::string> ParametersContainer::get_multiple_strings(const std::string &key, const std::vector<std::string> defaultValue) const
{
  return MapUtil::lookup(m_container, key, defaultValue);
}

std::string ParametersContainer::get_string(const std::string &key) const
{
  return get_typed_value<std::string>(key);
}

std::string ParametersContainer::get_string(const std::string &key, const std::string &defaultValue) const
{
  return get_typed_value<std::string>(key, defaultValue);
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
