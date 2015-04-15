/**
 * spaint: LabelManager.cpp
 */

#include "util/LabelManager.h"

#include <algorithm>

#include <tvgutil/MapUtil.h>
using namespace tvgutil;

namespace {

//#################### LOCAL CONSTANTS ####################

/**
 * Kelly's colours of maximum contrast (see https://eleanormaclure.files.wordpress.com/2011/03/colour-coding.pdf),
 * excluding white and black.
 */
const Vector3u colours[] = {
  Vector3u(255, 179, 0),
  Vector3u(128, 62, 117),
  Vector3u(255, 104, 0),
  Vector3u(166, 189, 215),
  Vector3u(193, 0, 32),
  Vector3u(206, 162, 98),
  Vector3u(129, 112, 102),

  // The remaining colours aren't good for people with defective colour vision:
  Vector3u(0, 125, 52),
  Vector3u(246, 118, 142),
  Vector3u(0, 83, 138),
  Vector3u(255, 122, 92),
  Vector3u(83, 55, 122),
  Vector3u(255, 142, 0),
  Vector3u(179, 40, 81),
  Vector3u(244, 200, 0),
  Vector3u(127, 24, 13),
  Vector3u(147, 170, 0),
  Vector3u(89, 51, 21),
  Vector3u(241, 58, 19),
  Vector3u(35, 44, 22)
};

/** The number of available colours. */
const size_t AVAILABLE_COLOUR_COUNT = sizeof(colours) / sizeof(Vector3u);

}

namespace spaint {

//#################### CONSTRUCTORS ####################

LabelManager::LabelManager(size_t maxLabelCount)
: m_maxLabelCount(std::min<size_t>(maxLabelCount, AVAILABLE_COLOUR_COUNT))
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

bool LabelManager::add_label(const std::string& name)
{
  // Make sure that we're not trying to exceed the maximum number of labels.
  if(label_count() == max_label_count()) return false;

  // Add the new label.
  SpaintVoxel::LabelType label = static_cast<SpaintVoxel::LabelType>(m_labelAllocator.allocate());
  m_labelProperties.insert(std::make_pair(label, std::make_pair(name, colours[label])));
  m_labelsByName.insert(std::make_pair(name, label));

  return true;
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

SpaintVoxel::LabelType LabelManager::get_next_label(SpaintVoxel::LabelType label) const
{
  const std::set<int>& used = m_labelAllocator.used();
  std::set<int>::const_iterator it = used.upper_bound(static_cast<int>(label));
  return it != used.end() ? static_cast<SpaintVoxel::LabelType>(*it) : label;
}

SpaintVoxel::LabelType LabelManager::get_previous_label(SpaintVoxel::LabelType label) const
{
  const std::set<int>& used = m_labelAllocator.used();
  std::set<int>::const_iterator it = used.find(static_cast<int>(label));
  return it != used.begin() ? static_cast<SpaintVoxel::LabelType>(*--it) : label;
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
