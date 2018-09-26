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
protected:
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

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Reads a simple value from the specified byte segment in the message.
   *
   * \param segment The byte segment from which to read the value.
   * \return        The value.
   */
  template <typename T>
  T read_simple(const Segment& segment) const
  {
    return *reinterpret_cast<const T*>(&m_data[segment.first]);
  }

  /**
   * \brief Writes a simple value into the specified byte segment in the message.
   *
   * \param value   The value.
   * \param segment The byte segment into which to write it.
   */
  template <typename T>
  void write_simple(const T& value, const Segment& segment)
  {
    memcpy(&m_data[segment.first], reinterpret_cast<const char*>(&value), segment.second);
  }

  //#################### PROTECTED STATIC MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Gets the end offset of the specified message segment.
   *
   * \param segment The segment.
   * \return        The end offset of the segment.
   */
  static size_t end_of(const Segment& segment);
};

}

#endif
