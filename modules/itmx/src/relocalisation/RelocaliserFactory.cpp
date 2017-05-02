/**
 * itmx: RelocaliserFactory.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "relocalisation/RelocaliserFactory.h"

#include "relocalisation/FernRelocaliser.h"

namespace itmx {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

Relocaliser_Ptr RelocaliserFactory::make_custom_fern_relocaliser(Vector2i depthImageSize,
                                                                 float viewFrustumMin,
                                                                 float viewFrustumMax,
                                                                 float harvestingThreshold,
                                                                 int numFerns,
                                                                 int decisionsPerFern,
                                                                 FernRelocaliser::KeyframeAddPolicy keyframeAddPolicy)
{
  return Relocaliser_Ptr(new FernRelocaliser(depthImageSize,
                                             viewFrustumMin,
                                             viewFrustumMax,
                                             harvestingThreshold,
                                             numFerns,
                                             decisionsPerFern,
                                             keyframeAddPolicy));
}

Relocaliser_Ptr RelocaliserFactory::make_default_fern_relocaliser(Vector2i depthImageSize,
                                                                  float viewFrustumMin,
                                                                  float viewFrustumMax)
{
  return make_custom_fern_relocaliser(depthImageSize,
                                      viewFrustumMin,
                                      viewFrustumMax,
                                      FernRelocaliser::get_default_harvesting_threshold(),
                                      FernRelocaliser::get_default_num_ferns(),
                                      FernRelocaliser::get_default_num_decisions_per_fern(),
                                      FernRelocaliser::get_default_keyframe_add_policy());
}

} // namespace itmx
