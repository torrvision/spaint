/**
 * itmx: RefiningRelocaliser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_REFININGRELOCALISER
#define H_ITMX_REFININGRELOCALISER

#include "Relocaliser.h"

//#include <ITMLib/Objects/Tracking/ITMTrackingState.h>

namespace itmx {

/**
 * \brief An instance of this class allows performing camera relocalisation from RGB-D image pairs followed by a
 *        refinement step.
 */
class RefiningRelocaliser : public Relocaliser
{
  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets a pointer to the refined relocaliser.
   *
   * \return A pointer to the inner relocaliser.
   */
  virtual Relocaliser_Ptr get_inner_relocaliser() const = 0;

  /**
   * \brief Attempt to relocalise the location from which an RGB-D image pair is acquired.
   *        Provides more details on the relcalisation phase.
   *
   * \param colourImage     The colour image.
   * \param depthImage      The depth image.
   * \param depthIntrinsics The intrinsic parameters of the depth sensor.
   * \param initialPose     The camera pose estimated by the inner relocaliser if it succeeded, boost::none otherwise.
   *
   * \return The result of the relocalisation if successful, an empty optional otherwise.
   */
  virtual boost::optional<Result> relocalise(const ITMUChar4Image *colourImage,
                                             const ITMFloatImage *depthImage,
                                             const Vector4f& depthIntrinsics,
                                             boost::optional<ORUtils::SE3Pose>& initialPose) const = 0;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<RefiningRelocaliser> RefiningRelocaliser_Ptr;
typedef boost::shared_ptr<const RefiningRelocaliser> RefiningRelocaliser_CPtr;

}

#endif
