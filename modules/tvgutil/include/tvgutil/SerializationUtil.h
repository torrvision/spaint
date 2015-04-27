/**
 * tvgutil: SerializationUtil.h
 */

#ifndef H_TVGUTIL_SERIALIZATIONUTIL
#define H_TVGUTIL_SERIALIZATIONUTIL

#include <fstream>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/extended_type_info.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/singleton.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>

namespace tvgutil {

/**
 * \brief This struct provides utility functions for saving/loading objects to/from files on disk.
 */
struct SerializationUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Loads an object from a file in binary format.
   *
   * \param path  The path to the file.
   * \param ptr   A pointer to some memory into which to load the object.
   */
  template <typename T>
  static inline void load_binary(const std::string& path, T*& ptr)
  {
    std::ifstream fs(path.c_str());
    boost::archive::binary_iarchive ar(fs);
    ar >> ptr;
  }

  /**
   * \brief Loads an object from a file in text format.
   *
   * \param path  The path to the file.
   * \param ptr   A pointer to some memory into which to load the object.
   */
  template <typename T>
  static inline void load_text(const std::string& path, T*& ptr)
  {
    std::ifstream fs(path.c_str());
    boost::archive::text_iarchive ar(fs);
    ar >> ptr;
  }

  /**
   * \brief Saves an object to a file in binary format.
   *
   * \param path  The path to the file.
   * \param obj   The object to save.
   */
  template <typename T>
  static inline void save_binary(const std::string& path, const T& obj)
  {
    std::ofstream fs(path.c_str());
    boost::archive::binary_oarchive ar(fs);
    const T *ptr = &obj;
    ar << ptr;
  }

  /**
   * \brief Saves an object to a file in text format.
   *
   * \param path  The path to the file.
   * \param obj   The object to save.
   */
  template <typename T>
  static inline void save_text(const std::string& path, const T& obj)
  {
    std::ofstream fs(path.c_str());
    boost::archive::text_oarchive ar(fs);
    const T *ptr = &obj;
    ar << ptr;
  }
};

}

#endif
