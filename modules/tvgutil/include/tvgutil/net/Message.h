/**
 * tvgutil: Message.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_TVGUTIL_MESSAGE
#define H_TVGUTIL_MESSAGE

#include <cstddef>
#include <vector>

namespace tvgutil {

/**
 * \brief An instance of a class deriving from this one represents a message that can be sent across a network.
 */
class Message
{
  //#################### TYPEDEFS ####################
public:
  /** An (offset, size) pair used to specify a byte segment within the message data. */
  typedef std::pair<size_t,size_t> Segment;

  //#################### PROTECTED VARIABLES ####################
protected:
  /** The message data. */
  std::vector<char> m_data;

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the message.
   */
  virtual ~Message();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets a raw pointer to the message data.
   *
   * \return  A raw pointer to the message data.
   */
  char *get_data_ptr();

  /**
   * \brief Gets a raw pointer to the message data.
   *
   * \return  A raw pointer to the message data.
   */
  const char *get_data_ptr() const;

  /**
   * \brief Gets the size of the message.
   *
   * \return  The size of the message.
   */
  size_t get_size() const;
};

}

#endif
