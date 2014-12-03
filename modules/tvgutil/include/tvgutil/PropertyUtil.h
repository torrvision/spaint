/**
 * tvgutil: PropertyUtil.h
 */

#ifndef H_TVGUTIL_PROPERTYUTIL
#define H_TVGUTIL_PROPERTYUTIL

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
   * \brief Attempts to load a tree of properties from an XML file.
   *
   * \param filename                                The name of the file.
   * \return                                        The tree of properties.
   * \throws boost::property_tree::xml_parser_error If the load fails.
   */
  static boost::property_tree::ptree load_properties_from_xml(const std::string& filename);
};

}

#endif
