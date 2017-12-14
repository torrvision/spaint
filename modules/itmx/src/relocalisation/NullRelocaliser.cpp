/**
 * itmx: NullRelocaliser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "relocalisation/NullRelocaliser.h"

namespace itmx {

//#################### PUBLIC MEMBER FUNCTIONS ####################

void NullRelocaliser::load_from_disk(const std::string& inputFolder)
{
  // No-op
}

boost::optional<NullRelocaliser::Result>
NullRelocaliser::relocalise(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage, const Vector4f& depthIntrinsics) const
{
  // A null relocaliser always fails to relocalise.
  return boost::none;
}

void NullRelocaliser::reset()
{
  // No-op
}

void NullRelocaliser::save_to_disk(const std::string& outputFolder) const
{
  // No-op
}

void NullRelocaliser::train(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage, const Vector4f& depthIntrinsics, const ORUtils::SE3Pose& cameraPose)
{
  // No-op
}

}
