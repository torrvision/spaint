/**
 * tvgutil: CommandManager.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_TVGUTIL_COMMANDMANAGER
#define H_TVGUTIL_COMMANDMANAGER

#include <climits>
#include <deque>
#include <map>

#include "Command.h"

namespace tvgutil {

/**
 * \brief An instance of this class can be used to manage the execution/undo/redo of commands.
 */
class CommandManager
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** A stack containing commands that have been executed and not undone. */
  std::deque<Command_CPtr> m_executed;

  /** The maximum size of the command history (the maximum combined size of the two command stacks). */
  size_t m_maxHistorySize;

  /** A stack containing commands that have been undone. */
  std::deque<Command_CPtr> m_undone;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a command manager.
   *
   * \param maxHistorySize  The maximum size of the command history (the maximum combined size of the two command stacks).
   */
  explicit CommandManager(size_t maxHistorySize = INT_MAX);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets whether or not there is currently a command that can be redone.
   *
   * \return  true, if there is currently a command that can be redone, or false otherwise.
   */
  bool can_redo() const;

  /**
   * \brief Gets whether or not there is currently a command that can be undone.
   *
   * \return  true, if there is currently a command that can be undone, or false otherwise.
   */
  bool can_undo() const;

  /**
   * \brief Executes the specified command.
   *
   * \param c The command to execute.
   */
  void execute_command(const Command_CPtr& c);

  /**
   * \brief Executes the specified command, compressing it into the previous command as necessary.
   *
   * If the executed stack is non-empty and the description of the most recent command matches one
   * of the specified precursors, then we compress the previous command and this one into a single
   * command. Otherwise, we just execute the specified command as is. The idea is to avoid making
   * undo/redo more user-intensive than it needs to be.
   *
   * \param c           The command to execute.
   * \param precursors  A map containing possible precursors of the current command for compression purposes.
   *                    If the command preceding the current command has a description that matches the key
   *                    of an element in the precursors map, then the two commands are compressed into a
   *                    single command whose description equals the corresponding value in the map.
   */
  void execute_compressible_command(const Command_CPtr& c, const std::map<std::string,std::string>& precursors);

  /**
   * \brief Gets the number of commands currently on the executed stack.
   *
   * \return  The number of commands currently on the executed stack.
   */
  size_t executed_count() const;

  /**
   * \brief Redoes the last command undone, if any.
   */
  void redo();

  /**
   * \brief Resets the command manager (clears the executed and undone command stacks).
   */
  void reset();

  /**
   * \brief Undoes the last command executed, if any.
   */
  void undo();

  /**
   * \brief Gets the number of commands currently on the undone stack.
   *
   * \return  The number of commands currently on the undone stack.
   */
  size_t undone_count() const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Makes space for a new command if the command history is full.
   */
  void make_space_for_command();
};

}

#endif
