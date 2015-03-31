/**
 * tvgutil: CommandManager.cpp
 */

#include "commands/CommandManager.h"

#include "commands/SeqCommand.h"

namespace tvgutil {

//#################### PUBLIC MEMBER FUNCTIONS ####################

bool CommandManager::can_redo() const
{
  return !m_undone.empty();
}

bool CommandManager::can_undo() const
{
  return !m_executed.empty();
}

void CommandManager::execute_command(const Command_CPtr& c)
{
  m_executed.push_back(c);
  m_undone.clear();
  c->execute();
}

void CommandManager::execute_compressible_command(const Command_CPtr& c, const std::map<std::string,std::string>& precursors)
{
  if(m_executed.empty())
  {
    execute_command(c);
  }
  else
  {
    Command_CPtr last = m_executed.back();
    const std::string& lastDescription = last->get_description();
    std::map<std::string,std::string>::const_iterator it = precursors.find(lastDescription);
    if(it != precursors.end())
    {
      m_executed.pop_back();
      m_executed.push_back(Command_CPtr(new SeqCommand(last, c, it->second)));
      m_undone.clear();
      c->execute();
      return;
    }

    // If we get here, we couldn't combine the last command with this one, so just execute the command that was passed in as normal.
    execute_command(c);
  }
}

size_t CommandManager::executed_count() const
{
  return m_executed.size();
}

void CommandManager::redo()
{
  if(can_redo())
  {
    Command_CPtr c = m_undone.back();
    m_undone.pop_back();
    m_executed.push_back(c);
    c->execute();
  }
}

void CommandManager::reset()
{
  m_executed.clear();
  m_undone.clear();
}

void CommandManager::undo()
{
  if(can_undo())
  {
    Command_CPtr c = m_executed.back();
    m_executed.pop_back();
    m_undone.push_back(c);
    c->undo();
  }
}

size_t CommandManager::undone_count() const
{
  return m_undone.size();
}

}
