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
  // Magic parameters from InfiniTAM.
  const float harvestingThreshold = 0.2f;
  const int numFerns = 500;
  const int numDecisionsPerFern = 4;
  const FernRelocaliser::KeyframeAddPolicy keyframePolicy = FernRelocaliser::DELAY_AFTER_RELOCALISATION;

  return make_custom_fern_relocaliser(depthImageSize,
                                      viewFrustumMin,
                                      viewFrustumMax,
                                      harvestingThreshold,
                                      numFerns,
                                      numDecisionsPerFern,
                                      keyframePolicy);
}

} // namespace itmx
