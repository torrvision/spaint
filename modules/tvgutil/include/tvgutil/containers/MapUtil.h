/**
 * tvgutil: MapUtil.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_TVGUTIL_MAPUTIL
#define H_TVGUTIL_MAPUTIL

#include <map>
#include <stdexcept>
#include <string>

#include <boost/function.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/mpl/identity.hpp>

namespace tvgutil {

/**
 * \brief This class provides utility functions related to std::map.
 */
class MapUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Applies a function to the value associated with a particular key iff it exists in the map.
   *
   * \param map   The map.
   * \param key   The key.
   * \param func  The function to apply.
   */
  template <typename K, typename V>
  static void call_if_found(std::map<K,V>& map, typename boost::mpl::identity<const K>::type& key, typename boost::mpl::identity<const boost::function<void(V&)> >::type& func)
  {
    typename std::map<K,V>::iterator it = map.find(key);
    if(it != map.end()) func(it->second);
  }

  /**
   * \brief Applies a function to the value associated with a particular key iff it exists in the map.
   *
   * \param map   The map.
   * \param key   The key.
   * \param func  The function to apply.
   */
  template <typename K, typename V>
  static void call_if_found(const std::map<K,V>& map, typename boost::mpl::identity<const K>::type& key, typename boost::mpl::identity<const boost::function<void(const V&)> >::type& func)
  {
    typename std::map<K,V>::const_iterator it = map.find(key);
    if(it != map.end()) func(it->second);
  }

  /**
   * \brief Gets the value associated with a particular key in the map.
   *
   * \param map                 The map.
   * \param key                 The key.
   * \return                    The value.
   * \throws std::runtime_error If the map does not contain the specified key.
   */
  template <typename K, typename V>
  static V& lookup(std::map<K,V>& map, typename boost::mpl::identity<const K>::type& key)
  {
    typename std::map<K,V>::iterator it = map.find(key);
    if(it != map.end()) return it->second;
    else throw std::runtime_error("The map does not contain the specified key");
  }

  /**
   * \brief Gets the value associated with a particular key in the map.
   *
   * \param map                 The map.
   * \param key                 The key.
   * \return                    The value.
   * \throws std::runtime_error If the map does not contain the specified key.
   */
  template <typename K, typename V>
  static const V& lookup(const std::map<K,V>& map, typename boost::mpl::identity<const K>::type& key)
  {
    typename std::map<K,V>::const_iterator it = map.find(key);
    if(it != map.end()) return it->second;
    else throw std::runtime_error("The map does not contain the specified key");
  }

  /**
   * \brief Gets the value associated with a particular key in the map, if any, or a default value otherwise.
   *
   * \param map                 The map.
   * \param key                 The key.
   * \param defaultValue        A default value to return if the key is not present in the map.
   * \return                    The value associated with the key in the map, if any, or the default value otherwise.
   */
  template <typename K, typename V>
  static const V& lookup(const std::map<K,V>& map, typename boost::mpl::identity<const K>::type& key, typename boost::mpl::identity<const V>::type& defaultValue)
  {
    typename std::map<K,V>::const_iterator it = map.find(key);
    return it != map.end() ? it->second : defaultValue;
  }

  /**
   * \brief Writes the value associated with a particular key in a map into a typed variable.
   *
   * This succeeds precisely when the key exists in the map and its corresponding value is of a type that can
   * be converted to the type of the variable.
   *
   * \param map                       The map.
   * \param key                       The key.
   * \param value                     The variable into which to write the value.
   * \throws std::runtime_error       If the map does not contain the specified key.
   * \throws boost::bad_lexical_cast  If the corresponding value is of a type that cannot be converted to the type of the variable.
   */
  template <typename K, typename V, typename T>
  static void typed_lookup(const std::map<K,V>& map, typename boost::mpl::identity<const K>::type& key, T& value)
  {
    value = boost::lexical_cast<T>(lookup(map, key));
  }
};

}

#endif
