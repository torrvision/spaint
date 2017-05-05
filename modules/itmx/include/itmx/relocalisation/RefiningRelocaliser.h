/**
 * itmx: RefiningRelocaliser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_REFININGRELOCALISER
#define H_ITMX_REFININGRELOCALISER

#include "Relocaliser.h"

#include <ITMLib/Objects/Tracking/ITMTrackingState.h>

namespace itmx {

/**
 * \brief An instance of this class allows performing camera relocalisation from RGB-D image pairs followed by a
 *        refinement step.
 */
class RefiningRelocaliser : public Relocaliser
{
  //#################### NESTED TYPES ####################
public:
  /**
   * \brief An instance of this struct will contain details on the intermediate relocalisation and refinement steps.
   */
  struct RefinementDetails
  {
    /** The initial pose estimated by the relocaliser. */
    boost::optional<ORUtils::SE3Pose> initialPose;

    /** Quality of the refinement tracking operation. */
    ITMLib::ITMTrackingState::TrackingResult refinementResult;
  };

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a RefiningRelocaliser.
   */
  RefiningRelocaliser();

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets a pointer to the refined relocaliser.
   *
   * \return A pointer to the inner relocaliser.
   */
  virtual Relocaliser_Ptr get_inner_relocaliser() const = 0;

  /**
   * \brief Integrates a newly acquired RGB-D image pair into the relocalisation system at a certain pose in the world.
   *
   * \param colourImage     The colour image.
   * \param depthImage      The depth image.
   * \param depthIntrinsics The intrinsic parameters of the depth sensor.
   * \param cameraPose      The position of the camera in the world.
   */
  virtual void integrate_rgbd_pose_pair(const ITMUChar4Image *colourImage,
                                        const ITMFloatImage *depthImage,
                                        const Vector4f &depthIntrinsics,
                                        const ORUtils::SE3Pose &cameraPose) = 0;

  /**
   * \brief Attempt to relocalise the location from which an RGB-D image pair is acquired.
   *
   * \param colourImage     The colour image.
   * \param depthImage      The depth image.
   * \param depthIntrinsics The intrinsic parameters of the depth sensor.
   *
   * \return The camera pose if successful, an empty optional otherwise.
   */
  virtual boost::optional<ORUtils::SE3Pose> relocalise(const ITMUChar4Image *colourImage,
                                                       const ITMFloatImage *depthImage,
                                                       const Vector4f &depthIntrinsics) = 0;

  /**
   * \brief Attempt to relocalise the location from which an RGB-D image pair is acquired.
   *        Provides more details on the relcalisation phase.
   *
   * \param colourImage       The colour image.
   * \param depthImage        The depth image.
   * \param depthIntrinsics   The intrinsic parameters of the depth sensor.
   * \param refinementDetails A details structure that will be filled with informations captured during the
   *                          relocalisation.
   *
   * \return The camera pose if successful, an empty optional otherwise.
   */
  virtual boost::optional<ORUtils::SE3Pose> relocalise(const ITMUChar4Image *colourImage,
                                                       const ITMFloatImage *depthImage,
                                                       const Vector4f &depthIntrinsics,
                                                       RefinementDetails &refinementDetails) = 0;

  /**
   * \brief Resets the relocaliser allowing the integration of informations on a new area.
   */
  virtual void reset() = 0;

  /**
   * \brief Updates the contents of the relocaliser when spare processing time is available. Can perform bookkeeping
   *        operations.
   */
  virtual void update() = 0;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<RefiningRelocaliser> RefiningRelocaliser_Ptr;
typedef boost::shared_ptr<const RefiningRelocaliser> RefiningRelocaliser_CPtr;

} // namespace itmx

#endif // H_ITMX_REFININGRELOCALISER
