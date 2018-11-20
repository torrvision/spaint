/**
 * orx: Relocaliser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "relocalisation/Relocaliser.h"

namespace orx {

//#################### CONSTRUCTORS ####################

Relocaliser::Relocaliser()
: m_timersEnabled(false)
{}

//#################### DESTRUCTOR ####################

Relocaliser::~Relocaliser() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void Relocaliser::finish_training()
{
  // No-op by default
}

ORUChar4Image_CPtr Relocaliser::get_visualisation_image(const std::string& key) const
{
  return ORUChar4Image_CPtr();
}

void Relocaliser::update()
{
  // No-op by default
}

//#################### PROTECTED MEMBER FUNCTIONS ####################

void Relocaliser::start_timer_nosync(AverageTimer& timer) const
{
  if(m_timersEnabled) timer.start_nosync();
}

void Relocaliser::start_timer_sync(AverageTimer& timer) const
{
  if(m_timersEnabled) timer.start_sync();
}

void Relocaliser::stop_timer_nosync(AverageTimer& timer) const
{
  if(m_timersEnabled) timer.stop_nosync();
}

void Relocaliser::stop_timer_sync(AverageTimer& timer) const
{
  if(m_timersEnabled) timer.stop_sync();
}

}
