/**
 * itmx: RelocaliserFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_RELOCALISERFACTORY
#define H_ITMX_RELOCALISERFACTORY

#include "FernRelocaliser.h"
#include "Relocaliser.h"

namespace itmx {

/**
 * \brief This class allows the construction of a Relocaliser.
 */
class RelocaliserFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Create a FernRelocaliser with custom parameters.
   *
   * \param depthImageSize      The dimensions of the depth images that will be passed to the relocaliser.
   * \param viewFrustumMin      The minimum distance to consider in the depth images.
   * \param viewFrustumMax      The maximum distance to consider in the depth images.
   * \param harvestingThreshold The threshold used when deciding whether to store a keyframe.
   * \param numFerns            The number of ferns used for relocalisation.
   * \param decisionsPerFern    The number of decisions that are performed in each fern.
   * \param keyframeAddPolicy   The policy to adapt to decide whether to store keyframes right after tracking failures.
   */
  static Relocaliser_Ptr make_custom_fern_relocaliser(Vector2i depthImageSize,
                                                      float viewFrustumMin,
                                                      float viewFrustumMax,
                                                      float harvestingThreshold,
                                                      int numFerns,
                                                      int decisionsPerFern,
                                                      FernRelocaliser::KeyframeAddPolicy keyframeAddPolicy);

  /**
   * \brief Create a FernRelocaliser with default parameters.
   *
   * \return An instance of Relocaliser.
   */
  static Relocaliser_Ptr
      make_default_fern_relocaliser(Vector2i depthImageSize, float viewFrustumMin, float viewFrustumMax);
};

} // namespace itmx

#endif // H_ITMX_RELOCALISERFACTORY
