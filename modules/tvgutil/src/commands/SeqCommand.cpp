/**
 * tvgutil: SeqCommand.cpp
 */

#include "commands/SeqCommand.h"

#include <stdexcept>

namespace tvgutil {

//#################### CONSTRUCTORS ####################

SeqCommand::SeqCommand(const Command_CPtr& c1, const Command_CPtr& c2, const std::string& description)
: Command(description)
{
  if(!c1 || !c2) throw std::runtime_error("Cannot construct a sequence command from a sequence containing a null command");
  m_cs.push_back(c1);
  m_cs.push_back(c2);
}

SeqCommand::SeqCommand(const std::vector<Command_CPtr>& cs, const std::string& description)
: Command(description), m_cs(cs)
{
  for(std::vector<Command_CPtr>::const_iterator it = m_cs.begin(), iend = m_cs.end(); it != iend; ++it)
  {
    if(!*it) throw std::runtime_error("Cannot construct a sequence command from a sequence containing a null command");
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void SeqCommand::execute() const
{
  for(std::vector<Command_CPtr>::const_iterator it = m_cs.begin(), iend = m_cs.end(); it != iend; ++it)
  {
    (*it)->execute();
  }
}

void SeqCommand::undo() const
{
  for(std::vector<Command_CPtr>::const_reverse_iterator it = m_cs.rbegin(), iend = m_cs.rend(); it != iend; ++it)
  {
    (*it)->undo();
  }
}

}
