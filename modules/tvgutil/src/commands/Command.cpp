/**
 * tvgutil: Command.cpp
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

void Command::execute() const
{
  // No-op by default
}

const std::string& Command::get_description() const
{
  return m_description;
}

void Command::undo() const
{
  // No-op by default
}

}
