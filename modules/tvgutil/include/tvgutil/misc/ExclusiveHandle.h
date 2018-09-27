/**
 * tvgutil: ExclusiveHandle.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_TVGUTIL_EXCLUSIVEHANDLE
#define H_TVGUTIL_EXCLUSIVEHANDLE

#include <boost/thread.hpp>

namespace tvgutil {

//#################### MAIN TYPE ####################

/**
 * \brief An instance of an instantiation of this class template can be used to provide exclusive access to an object.
 *
 * The lock will continue to be held until the handle is destroyed, at which point it will be released.
 */
template <typename T>
class ExclusiveHandle
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The lock used to provide exclusive access to the object. */
  boost::lock_guard<boost::mutex> m_lock;

  /** The object to which exclusive access is being provided. */
  T& m_object;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an exclusive handle to an object.
   *
   * \param object  The object to which exclusive access is being provided.
   * \param mutex   The mutex controlling access to the object.
   */
  ExclusiveHandle(T& object, boost::mutex& mutex)
  : m_lock(mutex), m_object(object)
  {}

  //#################### COPY CONSTRUCTOR & ASSIGNMENT OPERATOR ####################
private:
  // Deliberately private and unimplemented.
  ExclusiveHandle(const ExclusiveHandle<T>&);
  ExclusiveHandle<T>& operator=(const ExclusiveHandle<T>&);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the object to which exclusive access is being provided.
   *
   * \return  The object to which exclusive access is being provided.
   */
  T& get()
  {
    return m_object;
  }

  /**
   * \brief Gets the object to which exclusive access is being provided.
   *
   * \return  The object to which exclusive access is being provided.
   */
  const T& get() const
  {
    return m_object;
  }
};

//#################### HELPER FUNCTIONS ####################

/**
 * \brief Makes an exclusive handle to the specified object.
 *
 * \param object  The object to which exclusive access is being provided.
 * \param mutex   The mutex controlling access to the object.
 */
template <typename T>
boost::shared_ptr<ExclusiveHandle<T> > make_exclusive_handle(T& object, boost::mutex& mutex)
{
  return boost::shared_ptr<ExclusiveHandle<T> >(new ExclusiveHandle<T>(object, mutex));
}

//#################### HELPER TYPES ####################

template <typename T>
struct ExclusiveHandle_Ptr
{
  typedef boost::shared_ptr<ExclusiveHandle<T> > Type;
};

}

#endif
