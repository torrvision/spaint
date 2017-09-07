/**
 * tvgutil: PropertyUtil.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "persistence/PropertyUtil.h"

namespace tvgutil {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

boost::property_tree::ptree PropertyUtil::load_properties_from_xml_file(const std::string& filename)
{
  boost::property_tree::ptree tree;
  read_xml(filename, tree);
  return tree;
}

boost::property_tree::ptree PropertyUtil::load_properties_from_xml_string(const std::string& xmlString)
{
  boost::property_tree::ptree tree;

  std::stringstream ss;
  ss << xmlString;
  read_xml(ss, tree);

  return tree;
}

std::map<std::string,std::string> PropertyUtil::make_property_map(const boost::property_tree::ptree& tree)
{
  std::map<std::string,std::string> result;
  for(boost::property_tree::ptree::const_iterator it = tree.begin(), iend = tree.end(); it != iend; ++it)
  {
    std::string value;
    get_required_property<std::string>(tree, it->first, value);
    result.insert(std::make_pair(it->first, value));
  }
  return result;
}

}
