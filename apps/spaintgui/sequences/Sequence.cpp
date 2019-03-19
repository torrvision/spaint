/**
 * spaintgui: Sequence.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "Sequence.h"
namespace bf = boost::filesystem;

//#################### CONSTRUCTORS ####################

Sequence::Sequence(size_t initialFrameNumber)
: m_initialFrameNumber(initialFrameNumber)
{}

//#################### DESTRUCTOR ####################

Sequence::~Sequence() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

bf::path Sequence::default_calib_path() const
{
  return dir() / "calib.txt";
}

std::string Sequence::id() const
{
  return dir().stem().string();
}

//#################### STREAM OPERATORS ####################

std::ostream& operator<<(std::ostream& os, const Sequence& sequence)
{
  os << sequence.to_string();
  return os;
}
