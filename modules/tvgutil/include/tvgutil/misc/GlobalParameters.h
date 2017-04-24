/**
 * tvgutil: GlobalParameters.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_TVGUTIL_GLOBALPARAMETERS
#define H_TVGUTIL_GLOBALPARAMETERS

#include <ostream>
#include <vector>

#include "../containers/MapUtil.h"
#include "ConversionUtil.h"

namespace tvgutil {

/**
 * \brief This class can be used to access the global parameters used to configure an application.
 *
 * The parameters are represented as a key -> [value] map, i.e. there can be multiple values for the same named parameter.
 */
class GlobalParameters
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The key -> [value] map storing the values for the parameters. */
  std::map<std::string,std::vector<std::string> > m_params;

  //#################### SINGLETON IMPLEMENTATION ####################
private:
  /**
   * \brief Constructs the singleton instance.
   */
  GlobalParameters();

public:
  /**
   * \brief Gets the singleton instance.
   *
   * \return The singleton instance.
   */
  static GlobalParameters& instance();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Adds a value to the list of values for the specified parameter.
   *
   * \param key   The name of the parameter with which to associate the value.
   * \param value The value to add.
   */
  void add_value(const std::string& key, const std::string& value);

  /**
   * \brief Gets the first value associated with the specified parameter and converts it to the specified type.
   *
   * \param key The name of the parameter whose values are to be looked up.
   * \return    The first value associated with the specified parameter.
   *
   * \throws std::runtime_error       If the specified parameter does not exist.
   * \throws boost::bad_lexical_cast  If the parameter exists but the first value cannot be converted to the specified type.
   */
  template <typename T>
  T get_first_value(const std::string& key) const
  {
    const std::vector<std::string>& values = MapUtil::lookup(m_params, key);
    if(values.empty()) throw std::runtime_error("Value for " + key + " not found in the container");
    return from_string<T>(values[0]);
  }

  /**
   * \brief Gets the first value associated with the specified parameter and converts it to the specified type.
   *        If no such parameter exists, the specified default value is returned.
   *
   * \param key           The name of the parameter whose values are to be looked up.
   * \param defaultValue  The default value to return if the specified parameter does not exist.
   * \return              The first value associated with the specified parameter, if the parameter exists, or the default value otherwise.
   *
   * \throws boost::bad_lexical_cast  If the parameter exists but the first value cannot be converted to the specified type.
   */
  template <typename T>
  T get_first_value(const std::string& key, typename boost::mpl::identity<const T>::type& defaultValue) const
  {
    static std::vector<std::string> defaultEmptyVector;
    const std::vector<std::string>& values = MapUtil::lookup(m_params, key, defaultEmptyVector);
    return values.empty() ? defaultValue : from_string<T>(values[0]);
  }

  //#################### STREAM OPERATORS ####################
public:
  /**
   * \brief Outputs the global parameters to a stream.
   *
   * \param os  The stream.
   * \param rhs The global parameters to output.
   * \return    The stream.
   */
  friend std::ostream& operator<<(std::ostream& os, const GlobalParameters& rhs);
};

}

#endif
