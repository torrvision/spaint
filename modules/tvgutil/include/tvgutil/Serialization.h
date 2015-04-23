/**
 * tvgutil: Serialization.h
 */

#ifndef H_TVGUTIL_SERIALIZATION
#define H_TVGUTIL_SERIALIZATION

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
 * \brief Loads an object from a file.
 *
 * \param path  The path to the file.
 * \param ptr   A pointer to some memory into which to load the object.
 */
template <typename T>
inline void boost_serial_load(const std::string& path, T **ptr)
{
  std::ifstream fs(path.c_str());
  boost::archive::text_iarchive ar(fs);
  ar >> *ptr;
}

/**
 * \brief Saves an object to a file.
 *
 * \param path  The path to the file.
 * \param ptr   A pointer to the object to save.
 */
template <typename T>
inline void boost_serial_save(const std::string& path, const T *ptr)
{
  std::ofstream fs(path.c_str());
  boost::archive::text_oarchive ar(fs);
  ar << ptr;
}

}

#endif
