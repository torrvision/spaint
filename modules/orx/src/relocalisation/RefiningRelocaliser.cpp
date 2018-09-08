/**
 * orx: RefiningRelocaliser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "relocalisation/RefiningRelocaliser.h"

namespace orx {

//#################### CONSTRUCTORS ####################

RefiningRelocaliser::RefiningRelocaliser(const Relocaliser_Ptr& innerRelocaliser)
: m_innerRelocaliser(innerRelocaliser)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

const Relocaliser_Ptr& RefiningRelocaliser::get_inner_relocaliser()
{
  return m_innerRelocaliser;
}

Relocaliser_CPtr RefiningRelocaliser::get_inner_relocaliser() const
{
  return m_innerRelocaliser;
}

}
