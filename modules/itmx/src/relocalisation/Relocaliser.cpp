/**
 * itmx: Relocaliser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "relocalisation/Relocaliser.h"

namespace itmx {

//#################### DESTRUCTOR ####################

Relocaliser::~Relocaliser() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void Relocaliser::finish_training()
{
  // No-op by default
}

void Relocaliser::load_from_disk(const std::string& inputFolder)
{
  // No-op by default
}

void Relocaliser::save_to_disk(const std::string& outputFolder) const
{
  // No-op by default
}

void Relocaliser::update()
{
  // No-op by default
}

}
