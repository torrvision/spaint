/**
 * tvgutil: ParametersContainer.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_TVGUTIL_PARAMETERSCONTAINER
#define H_TVGUTIL_PARAMETERSCONTAINER

#include <map>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <boost/lexical_cast.hpp>
#include <boost/mpl/identity.hpp>

#include "MapUtil.h"

namespace tvgutil {

/**
 * \brief An instance of this class can be used to hold a number of parameters used to configure the application.
 *
 * The container acts as a Key-Value storage returning typed variables.
 */
class ParametersContainer
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The actual container. */
  std::map<std::string, std::vector<std::string> > m_container;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a parameter container that can be used to hold a number of parameters used to configure the application.
   */
  ParametersContainer();

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Returns a global instance of a parameter container that can be used to configure deeply nested classes.
   *
   * \return A global instance of the parameter container.
   */
  static ParametersContainer& instance();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Adds a key-value pair to the container.
   *
   * Multiple values can be added to the same key.
   *
   * \param key   The key to which associate the value.
   * \param value The value to add to the container.
   */
  void add_value(const std::string &key, const std::string &value);

  /**
   * \brief Returns a typed value from the container.
   *
   * If multiple values have been added to the same key, the first is returned.
   *
   * \param key The key.
   * \return    The value.
   *
   * \throws std::runtime_error       If the container does not contain the specified key.
   * \throws boost::bad_lexical_cast  If the corresponding value in the container cannot be converted to the requested type.
   */
  template<typename T>
  T get_typed_value(const std::string &key) const
  {
    std::vector<std::string> values = MapUtil::lookup(m_container, key);

    if(values.empty())
      throw std::runtime_error("Value for " + key + " not found in the container.");

    return boost::lexical_cast<T>(values[0]);
  }

  /**
   * \brief Returns a typed value from the container.
   *
   * If multiple values have been added to the same key, the first is returned.
   * If the key is missing returns the default value.
   *
   * \param key          The key.
   * \param defaultValue The default value.
   * \return             The value.
   *
   * \throws boost::bad_lexical_cast  If the corresponding value in the container cannot be converted to the requested type.
   */
  template<typename T>
  T get_typed_value(const std::string &key, typename boost::mpl::identity<const T>::type &defaultValue) const
  {
    static std::vector<std::string> defaultEmptyVector;
    std::vector<std::string> values = MapUtil::lookup(m_container, key, defaultEmptyVector);

    return values.empty() ? defaultValue : boost::lexical_cast<T>(values[0]);
  }

  //#################### STREAM OPERATORS ####################
public:
  /**
   * \brief Outputs the contents of the container.
   *
   * \param os  The stream to which to output the container.
   * \param rhs The container to output.
   * \return    The stream.
   */
  friend std::ostream& operator<<(std::ostream &os, const ParametersContainer &rhs);
};

//#################### TEMPLATE SPECIALIZATIONS ####################

/**
 * \brief Returns a typed value from the container.
 *
 * Specialization for bool since lexical_cast does not handle "true" and "false".
 * If multiple values have been added to the same key, the first is returned.
 *
 * \param key The key.
 * \return    The value.
 *
 * \throws std::runtime_error       If the container does not contain the specified key.
 * \throws boost::bad_lexical_cast  If the corresponding value in the container cannot be converted to the requested type.
 */
template<>
inline bool ParametersContainer::get_typed_value<bool>(const std::string &key) const
{
  std::vector<std::string> values = MapUtil::lookup(m_container, key);

  if(values.empty())
    throw std::runtime_error("Value for " + key + " not found in the container.");

  bool value;
  std::istringstream ss(values[0]);
  ss >> std::boolalpha >> value;

  return value;
}

/**
 * \brief Returns a typed value from the container.
 *
 * Specialization for bool since lexical_cast does not handle "true" and "false".
 * If multiple values have been added to the same key, the first is returned.
 * If the key is missing returns the default value.
 *
 * \param key          The key.
 * \param defaultValue The default value.
 * \return             The value.
 *
 * \throws boost::bad_lexical_cast  If the corresponding value in the container cannot be converted to the requested type.
 */
template<>
inline bool ParametersContainer::get_typed_value<bool>(const std::string &key, typename boost::mpl::identity<const bool>::type &defaultValue) const
{
  static std::vector<std::string> defaultEmptyVector;
  std::vector<std::string> values = MapUtil::lookup(m_container, key, defaultEmptyVector);

  bool value = defaultValue;

  if(!values.empty())
  {
    std::istringstream ss(values[0]);
    ss >> std::boolalpha >> value;
  }

  return value;
}

}

#endif
