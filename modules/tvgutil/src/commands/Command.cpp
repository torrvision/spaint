/**
 * tvgutil: Command.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "commands/Command.h"

namespace tvgutil {

//#################### CONSTRUCTORS ####################

Command::Command(const std::string& description)
: m_description(description)
{}

//#################### DESTRUCTOR ####################

Command::~Command() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

const std::string& Command::get_description() const
{
  return m_description;
}

}
