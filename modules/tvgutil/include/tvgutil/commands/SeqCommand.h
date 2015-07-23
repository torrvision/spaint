/**
 * tvgutil: SeqCommand.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_TVGUTIL_SEQCOMMAND
#define H_TVGUTIL_SEQCOMMAND

#include <vector>

#include "Command.h"

namespace tvgutil {

/**
 * \brief An instance of this class represents a command that is a sequence of one or more "smaller" commands.
 */
class SeqCommand : public Command
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The sequence of "smaller" commands. */
  std::vector<Command_CPtr> m_cs;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a command representing c1; c2.
   *
   * \pre                       Neither of the commands may be NULL.
   * \param c1                  The command to execute first (and undo second).
   * \param c2                  The command to execute second (and undo first).
   * \param description         The description to give the sequence command.
   * \throws std::runtime_error If either of the commands is NULL.
   */
  SeqCommand(const Command_CPtr& c1, const Command_CPtr& c2, const std::string& description);

  /**
   * \brief Constructs a command representing cs[0]; cs[1]; ...; cs[cs.size()-1].
   *
   * \pre                       None of the commands may be NULL.
   * \param cs                  The commands to execute (in the order in which they should be executed).
   * \param description         The description to give the sequence command.
   * \throws std::runtime_error If any of the commands are NULL.
   */
  SeqCommand(const std::vector<Command_CPtr>& cs, const std::string& description);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void execute() const;

  /** Override */
  virtual void undo() const;
};

}

#endif
