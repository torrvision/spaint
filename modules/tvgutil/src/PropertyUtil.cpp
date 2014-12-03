/**
 * tvgutil: PropertyUtil.cpp
 */

#include "PropertyUtil.h"

namespace tvgutil {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

boost::property_tree::ptree PropertyUtil::load_properties_from_xml(const std::string& filename)
{
  boost::property_tree::ptree tree;
  read_xml(filename, tree);
  return tree;
}

}
