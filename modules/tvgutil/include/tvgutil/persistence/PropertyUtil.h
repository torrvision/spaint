/**
 * tvgutil: PropertyUtil.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_TVGUTIL_PROPERTYUTIL
#define H_TVGUTIL_PROPERTYUTIL

#include <map>
#include <string>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

namespace tvgutil {

/**
 * \brief This struct contains utility functions to make it easier to work with Boost's property trees.
 */
struct PropertyUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Attempts to get a property from a property tree.
   *
   * If the property could not be found in the tree, this function has no effect.
   *
   * \param tree    The property tree.
   * \param path    The tree path to the desired property.
   * \param result  A variable into which to store the value of the property (if found).
   */
  template <typename T>
  static void get_optional_property(const boost::property_tree::ptree& tree, const std::string& path, T& result)
  {
    result = tree.get(path, result);
  }

  /**
   * \brief Attempts to get a property from a property tree.
   *
   * \param tree                                  The property tree.
   * \param path                                  The tree path to the desired property.
   * \param result                                A variable into which to store the value of the property (if found).
   * \throws boost::property_tree::ptree_bad_path If the property could not be found in the tree.
   */
  template <typename T>
  static void get_required_property(const boost::property_tree::ptree& tree, const std::string& path, T& result)
  {
    result = tree.get<T>(path);
  }

  /**
   * \brief Attempts to load a property tree from an XML file.
   *
   * \param filename                                The name of the file.
   * \return                                        The property tree.
   * \throws boost::property_tree::xml_parser_error If the load fails.
   */
  static boost::property_tree::ptree load_properties_from_xml_file(const std::string& filename);

  /**
   * \brief Attempts to load a property tree from a string containing XML.
   *
   * \param xmlString                               The string containing XML.
   * \return                                        The property tree.
   * \throws boost::property_tree::xml_parser_error If the load fails.
   */
  static boost::property_tree::ptree load_properties_from_xml_string(const std::string& xmlString);

  /**
   * \brief Converts a property tree into a property map.
   *
   * \param tree  The property tree.
   * \return      The property map.
   */
  static std::map<std::string,std::string> make_property_map(const boost::property_tree::ptree& tree);
};

}

#endif
