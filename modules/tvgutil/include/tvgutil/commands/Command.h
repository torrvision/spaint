/**
 * tvgutil: Command.h
 */

#ifndef H_TVGUTIL_COMMAND
#define H_TVGUTIL_COMMAND

#include <string>

#include <boost/shared_ptr.hpp>

namespace tvgutil {

/**
 * \brief An instance of a class deriving from this one can be used to represent a command that can be executed/undone/redone.
 */
class Command
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** A short description of what the command does. */
  std::string m_description;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a command.
   *
   * \param description A short description of what the command does.
   */
  explicit Command(const std::string& description);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the command.
   */
  virtual ~Command();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Executes the command.
   */
  virtual void execute() const;

  /**
   * \brief Gets a short description of what the command does.
   *
   * \return  A short description of what the command does.
   */
  const std::string& get_description() const;

  /**
   * \brief Undoes the command.
   */
  virtual void undo() const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<const Command> Command_CPtr;

}

#endif
