/**
 * spaint: ColourAppearanceModel.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "objects/ColourAppearanceModel.h"

namespace spaint {

//#################### PUBLIC MEMBER FUNCTIONS ####################

double ColourAppearanceModel::compute_posterior_probability(const Vector3u& rgbColour) const
{
  // TODO
  return 0.0;
}

void ColourAppearanceModel::update(const ITMUChar4Image_CPtr& image, const ITMUCharImage_CPtr& objectMask)
{
  // TODO
}

}
