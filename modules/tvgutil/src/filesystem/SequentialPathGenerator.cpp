/**
 * tvgutil: SequentialPathGenerator.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "filesystem/SequentialPathGenerator.h"

#include <boost/format.hpp>

namespace tvgutil {

//#################### CONSTRUCTORS ####################

SequentialPathGenerator::SequentialPathGenerator(const boost::filesystem::path& baseDir)
: m_baseDir(baseDir), m_index(0)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

const boost::filesystem::path& SequentialPathGenerator::get_base_dir() const
{
  return m_baseDir;
}

void SequentialPathGenerator::increment_index()
{
  ++m_index;
}

boost::filesystem::path SequentialPathGenerator::make_path(const std::string& pattern) const
{
  return m_baseDir / (boost::format(pattern) % m_index).str();
}

}
