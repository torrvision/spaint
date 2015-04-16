/**
 * tvgutil: NoOpCommand.h
 */

#ifndef H_TVGUTIL_NOOPCOMMAND
#define H_TVGUTIL_NOOPCOMMAND

#include "Command.h"

namespace tvgutil {

/**
 * \brief An instance of this class represents a "no-op" command, i.e. a command that does nothing.
 *
 * No-op commands are useful for tasks such as denoting the start/end of compressible command sequences.
 */
class NoOpCommand : public Command
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a "no-op" command.
   *
   * \param description A short description of what the command does.
   */
  explicit NoOpCommand(const std::string& description);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void execute() const;

  /** Override */
  virtual void undo() const;
};

}

#endif
