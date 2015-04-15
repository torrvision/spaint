/**
 * spaint: LabelManager.cpp
 */

#include "util/LabelManager.h"

#include <tvgutil/MapUtil.h>
using namespace tvgutil;

namespace spaint {

//#################### CONSTRUCTORS ####################

LabelManager::LabelManager(size_t maxLabelCount)
: m_maxLabelCount(maxLabelCount)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

bool LabelManager::add_label(const std::string& name)
{
  // Make sure that we're not trying to exceed the maximum number of labels.
  if(label_count() == max_label_count()) return false;

  // Add the new label.
  SpaintVoxel::LabelType label = static_cast<SpaintVoxel::LabelType>(m_labelAllocator.allocate());
  Vector3u colour; // TODO
  m_labelProperties.insert(std::make_pair(label, std::make_pair(name, colour)));
  m_labelsByName.insert(std::make_pair(name, label));
}

void LabelManager::delete_label(SpaintVoxel::LabelType label)
{
  std::map<SpaintVoxel::LabelType,std::pair<std::string,Vector3u> >::iterator it = m_labelProperties.find(label);
  if(it == m_labelProperties.end()) throw std::runtime_error("Cannot delete unknown label '" + boost::lexical_cast<std::string>(label) + "'");

  m_labelAllocator.deallocate(static_cast<int>(label));
  m_labelsByName.erase(it->second.first);
  m_labelProperties.erase(it);
}

SpaintVoxel::LabelType LabelManager::get_label(const std::string& name) const
{
  return MapUtil::lookup(m_labelsByName, name);
}

Vector3u LabelManager::get_label_colour(SpaintVoxel::LabelType label) const
{
  return MapUtil::lookup(m_labelProperties, label).second;
}

std::string LabelManager::get_label_name(SpaintVoxel::LabelType label) const
{
  return MapUtil::lookup(m_labelProperties, label).first;
}

bool LabelManager::has_label(SpaintVoxel::LabelType label) const
{
  return m_labelProperties.find(label) != m_labelProperties.end();
}

bool LabelManager::has_label(const std::string& name) const
{
  return m_labelsByName.find(name) != m_labelsByName.end();
}

size_t LabelManager::label_count() const
{
  return m_labelAllocator.used_count();
}

size_t LabelManager::max_label_count() const
{
  return m_maxLabelCount;
}

}
