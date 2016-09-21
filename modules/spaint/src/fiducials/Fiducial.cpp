/**
 * spaint: Fiducial.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "fiducials/Fiducial.h"

#include <stdexcept>

namespace spaint {

//#################### CONSTRUCTORS ####################

Fiducial::Fiducial(const std::string& id, const Vector3f& pos)
: m_id(id), m_pos(pos)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

const std::string& Fiducial::id() const
{
  return m_id;
}

const Vector3f& Fiducial::pos() const
{
  return m_pos;
}

void Fiducial::update(const Fiducial& newFiducial)
{
  if(m_id != newFiducial.m_id)
  {
    throw std::runtime_error("Error: Cannot update a fiducial using a fiducial with a different ID");
  }

  // For now, just overwrite the properties of this fiducial with those of the new fiducial.
  // (More sophisticated alternatives can be implemented later if necessary.)
  m_pos = newFiducial.m_pos;
}

}
