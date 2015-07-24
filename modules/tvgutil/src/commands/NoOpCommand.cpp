/**
 * tvgutil: NoOpCommand.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
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
