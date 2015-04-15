/**
 * tvgutil: NoOpCommand.cpp
 */

#include "commands/NoOpCommand.h"

namespace tvgutil {

//#################### CONSTRUCTORS ####################

NoOpCommand::NoOpCommand(const std::string& description)
: Command(description)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void NoOpCommand::execute() const {}

void NoOpCommand::undo() const {}

}
