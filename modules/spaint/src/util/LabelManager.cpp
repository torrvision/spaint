/**
 * spaint: LabelManager.cpp
 */

#include "util/LabelManager.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

LabelManager::LabelManager(size_t maxLabelCount)
: m_maxLabelCount(maxLabelCount)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void LabelManager::add_label(const std::string& name)
{
  // TODO
}

void LabelManager::delete_label(const std::string& name)
{
  // TODO
}

SpaintVoxel::LabelType LabelManager::get_label(const std::string& name) const
{
  // TODO
  throw 23;
}

void /* Colour */ LabelManager::get_label_colour(const std::string& name) const
{
  // TODO
  throw 23;
}

std::string LabelManager::get_label_name(SpaintVoxel::LabelType label) const
{
  // TODO
  throw 23;
}

bool LabelManager::has_label(SpaintVoxel::LabelType label) const
{
  const std::set<int>& used = m_labelAllocator.used();
  return used.find(static_cast<int>(label)) != used.end();
}

bool LabelManager::has_label(const std::string& name) const
{
  // TODO
  throw 23;
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
