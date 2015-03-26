/**
 * tvgutil: Serialization.h
 */

#include <fstream>

#include <boost/serialization/singleton.hpp>
#include <boost/serialization/extended_type_info.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#ifndef H_TVGUTIL_SERIALIZATION
#define H_TVGUTIL_SERIALIZATION

namespace tvgutil {

/**
 * \brief A function which allows loading of type from a binary file.
 */
template <typename T>
inline void boost_serial_load(const std::string& path, T *type)
{
  std::ifstream ifs(path);
  boost::archive::binary_iarchive ia(ifs);
  ia >> *type;
}

/**
 * \brief A function which allows saving a type to a binary file.
 */
template <typename T>
inline void boost_serial_save(const std::string& path, T *type)
{
  std::ofstream ofs(path);
  boost::archive::binary_oarchive oa(ofs);
  oa << *type;
}

}

#endif
