/**
 * tvgutil: SettingsContainer.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_TVGUTIL_SETTINGSCONTAINER
#define H_TVGUTIL_SETTINGSCONTAINER

#include <ostream>
#include <vector>

#include <boost/shared_ptr.hpp>

#include "../containers/MapUtil.h"
#include "ConversionUtil.h"

namespace tvgutil {

/**
 * \brief An instance of this class can be used to store named settings for an application.
 *
 * The settings are represented as a key -> [value] map, i.e. there can be multiple values for the same setting.
 */
class SettingsContainer
{
  //#################### CONSTANTS ####################
public:
  /** The value to use to indicate that a setting has not been set by the user and has no sensible default. */
  static const std::string NOT_SET;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The key -> [value] map storing the values for the settings. */
  std::map<std::string,std::vector<std::string> > m_settings;

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the settings container.
   */
  virtual ~SettingsContainer();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Adds a value to the list of values for the specified setting.
   *
   * \param key   The name of the setting with which to associate the value.
   * \param value The value to add.
   */
  void add_value(const std::string& key, const std::string& value);

  /**
   * \brief Gets the first value associated with the specified setting and converts it to the specified type.
   *
   * \param key The name of the setting whose values are to be looked up.
   * \return    The first value associated with the specified setting.
   *
   * \throws std::runtime_error       If the specified setting does not exist.
   * \throws boost::bad_lexical_cast  If the setting exists but the first value cannot be converted to the specified type.
   */
  template <typename T>
  T get_first_value(const std::string& key) const
  {
    const std::vector<std::string>& values = MapUtil::lookup(m_settings, key);
    if(values.empty() || values[0] == NOT_SET) throw std::runtime_error("Value for " + key + " not found in the container");
    return from_string<T>(values[0]);
  }

  /**
   * \brief Gets the first value associated with the specified setting and converts it to the specified type.
   *        If no such setting exists, the specified default value is returned.
   *
   * \param key           The name of the setting whose values are to be looked up.
   * \param defaultValue  The default value to return if the specified setting does not exist.
   * \return              The first value associated with the specified setting, if the setting exists, or the default value otherwise.
   *
   * \throws boost::bad_lexical_cast  If the setting exists but the first value cannot be converted to the specified type.
   */
  template <typename T>
  T get_first_value(const std::string& key, typename boost::mpl::identity<const T>::type& defaultValue) const
  {
    static std::vector<std::string> defaultEmptyVector;
    const std::vector<std::string>& values = MapUtil::lookup(m_settings, key, defaultEmptyVector);
    return values.empty() || values[0] == NOT_SET ? defaultValue : from_string<T>(values[0]);
  }

  //#################### STREAM OPERATORS ####################
public:
  /**
   * \brief Outputs the settings in a settings container to a stream.
   *
   * \param os  The stream.
   * \param rhs The settings container.
   * \return    The stream.
   */
  friend std::ostream& operator<<(std::ostream& os, const SettingsContainer& rhs);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<SettingsContainer> SettingsContainer_Ptr;
typedef boost::shared_ptr<const SettingsContainer> SettingsContainer_CPtr;

}

#endif
