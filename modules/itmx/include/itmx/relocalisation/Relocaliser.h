/**
 * itmx: Relocaliser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_RELOCALISER
#define H_ITMX_RELOCALISER

#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

#include <ITMLib/Utils/ITMImageTypes.h>
#include <ORUtils/SE3Pose.h>

namespace itmx {

/**
 * \brief An instance of this class allows performing camera relocalisation from RGB-D image pairs.
 */
class Relocaliser
{
  //#################### NESTED TYPES ####################
public:
  /**
   * \brief An instance of this struct will contain the results of a relocalisation call.
   */
  struct RelocalisationResult
  {
    //#################### ENUMS ####################
    /** An instance of this enum allows to specify whether the relocalisation gave good or poor results. */
    enum RelocalisationQuality { RELOCALISATION_GOOD, RELOCALISATION_POOR };

    //#################### MEMBER VARIABLES ####################
    /** The pose estimated by the relocaliser. */
    ORUtils::SE3Pose pose;

    /** Quality of the relocalisation. */
    RelocalisationQuality quality;
  };

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a Relocaliser.
   */
  Relocaliser();

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destructs a Relocaliser.
   */
  virtual ~Relocaliser();

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
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
   * \return The result of the relocalisation if successful, an empty optional otherwise.
   */
  virtual boost::optional<RelocalisationResult> relocalise(const ITMUChar4Image *colourImage,
                                                           const ITMFloatImage *depthImage,
                                                           const Vector4f &depthIntrinsics) = 0;

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

typedef boost::shared_ptr<Relocaliser> Relocaliser_Ptr;
typedef boost::shared_ptr<const Relocaliser> Relocaliser_CPtr;

} // namespace itmx

#endif // H_ITMX_RELOCALISER
