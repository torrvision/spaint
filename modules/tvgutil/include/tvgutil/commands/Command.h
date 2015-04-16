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
protected:
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

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Executes the command.
   */
  virtual void execute() const = 0;

  /**
   * \brief Undoes the command.
   */
  virtual void undo() const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets a short description of what the command does.
   *
   * \return  A short description of what the command does.
   */
  const std::string& get_description() const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<const Command> Command_CPtr;

}

#endif
