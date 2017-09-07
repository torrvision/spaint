/**
 * spaint: LabelManager.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "util/LabelManager.h"

#include <algorithm>

#include <boost/assign/list_of.hpp>
using boost::assign::list_of;

#include <tvgutil/containers/MapUtil.h>
using namespace tvgutil;

namespace {

//#################### LOCAL CONSTANTS ####################

/**
 * Kelly's colours of maximum contrast (see https://eleanormaclure.files.wordpress.com/2011/03/colour-coding.pdf), excluding black.
 */
const std::vector<Vector3u> colours = list_of
  (Vector3u(255, 255, 255))
  (Vector3u(255, 179, 0))
  (Vector3u(128, 62, 117))
  (Vector3u(255, 104, 0))
  (Vector3u(166, 189, 215))
  (Vector3u(193, 0, 32))
  (Vector3u(206, 162, 98))
  (Vector3u(129, 112, 102))

  // The remaining colours aren't good for people with defective colour vision:
  (Vector3u(0, 125, 52))
  (Vector3u(246, 118, 142))
  (Vector3u(0, 83, 138))
  (Vector3u(255, 122, 92))
  (Vector3u(83, 55, 122))
  (Vector3u(255, 142, 0))
  (Vector3u(179, 40, 81))
  (Vector3u(244, 200, 0))
  (Vector3u(127, 24, 13))
  (Vector3u(147, 170, 0))
  (Vector3u(89, 51, 21))
  (Vector3u(241, 58, 19))
  (Vector3u(35, 44, 22));

}

namespace spaint {

//#################### CONSTRUCTORS ####################

LabelManager::LabelManager(size_t maxLabelCount)
: m_maxLabelCount(std::min<size_t>(maxLabelCount, colours.size()))
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

bool LabelManager::add_label(const std::string& name)
{
  // Make sure that a label with the specified name does not already exist.
  if(has_label(name)) return false;

  // Make sure that we're not trying to exceed the maximum number of labels.
  if(get_label_count() == get_max_label_count()) return false;

  // Add the new label.
  SpaintVoxel::Label label = static_cast<SpaintVoxel::Label>(m_labelAllocator.allocate());
  m_labelProperties.insert(std::make_pair(label, std::make_pair(name, colours[label])));
  m_labelsByName.insert(std::make_pair(name, label));

  return true;
}

SpaintVoxel::Label LabelManager::get_label(const std::string& name) const
{
  return MapUtil::lookup(m_labelsByName, name);
}

Vector3u LabelManager::get_label_colour(SpaintVoxel::Label label) const
{
  return MapUtil::lookup(m_labelProperties, label).second;
}

const std::vector<Vector3u>& LabelManager::get_label_colours() const
{
  return colours;
}

size_t LabelManager::get_label_count() const
{
  return m_labelAllocator.used_count();
}

std::string LabelManager::get_label_name(SpaintVoxel::Label label) const
{
  return MapUtil::lookup(m_labelProperties, label).first;
}

size_t LabelManager::get_max_label_count() const
{
  return m_maxLabelCount;
}

SpaintVoxel::Label LabelManager::get_next_label(SpaintVoxel::Label label) const
{
  const std::set<int>& used = m_labelAllocator.used();
  std::set<int>::const_iterator it = used.upper_bound(static_cast<int>(label));
  return it != used.end() ? static_cast<SpaintVoxel::Label>(*it) : label;
}

SpaintVoxel::Label LabelManager::get_previous_label(SpaintVoxel::Label label) const
{
  const std::set<int>& used = m_labelAllocator.used();
  std::set<int>::const_iterator it = used.find(static_cast<int>(label));
  return it != used.begin() ? static_cast<SpaintVoxel::Label>(*--it) : label;
}

bool LabelManager::has_label(SpaintVoxel::Label label) const
{
  return m_labelProperties.find(label) != m_labelProperties.end();
}

bool LabelManager::has_label(const std::string& name) const
{
  return m_labelsByName.find(name) != m_labelsByName.end();
}

}
