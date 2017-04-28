/**
 * itmx: FernRelocaliser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_FERNRELOCALISER
#define H_ITMX_FERNRELOCALISER

#include "Relocaliser.h"

#include <FernRelocLib/Relocaliser.h>

namespace itmx {

/**
 * \brief An instance of this class allows performing camera relocalisation from RGB-D image pairs wrapping an instance
 *        of the Fern-Based InfiniTAM relocaliser.
 */
class FernRelocaliser : public Relocaliser
{
  //#################### ENUMS ####################
public:
  /**
   * Whether to always try to add a keyframe to the pose database or wait after a relocalisation is performed.
   * Can be used to avoid storing keyframes right after a tracking failure until we are sure that they are actually
   * good.
   */
  enum KeyframeAddPolicy { ALWAYS_TRY_ADD, DELAY_AFTER_RELOCALISATION };

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a FernRelocaliser.
   *
   * \param depthImageSize      The dimensions of the depth images that will be passed to the relocaliser.
   * \param viewFrustumMin      The minimum distance to consider in the depth images.
   * \param viewFrustumMax      The maximum distance to consider in the depth images.
   * \param harvestingThreshold The threshold used when deciding whether to store a keyframe.
   * \param numFerns            The number of ferns used for relocalisation.
   * \param decisionsPerFern    The number of decisions that are performed in each fern.
   * \param keyframeAddPolicy   The policy to adapt to decide whether to store keyframes right after tracking failures.
   */
  FernRelocaliser(Vector2i depthImageSize,
                  float viewFrustumMin,
                  float viewFrustumMax,
                  float harvestingThreshold,
                  int numFerns,
                  int decisionsPerFern,
                  KeyframeAddPolicy keyframeAddPolicy = DELAY_AFTER_RELOCALISATION);

  //#################### PUBLIC VIRTUAL MEMBER FUNCTIONS ####################
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
                                        const ORUtils::SE3Pose &cameraPose);

  /**
   * \brief Attempt to relocalise the location from which an RGB-D image pair is acquired.
   *
   * \param colourImage     The colour image.
   * \param depthImage      The depth image.
   * \param depthIntrinsics The intrinsic parameters of the depth sensor.
   *
   * \return The camera pose if successful, an empty optional otherwise.
   */
  virtual boost::optional<ORUtils::SE3Pose>
      relocalise(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage, const Vector4f &depthIntrinsics);

  /**
   * \brief Resets the relocaliser allowing the integration of informations on a new area.
   */
  virtual void reset();

  /**
   * \brief Updates the contents of the relocaliser when spare processing time is available. Can perform bookkeeping
   *        operations. Does nothing in this impementation.
   */
  virtual void update();

  //#################### PRIVATE TYPEDEFS ####################
private:
  typedef FernRelocLib::Relocaliser<float> WrappedRelocaliser;
  typedef boost::shared_ptr<WrappedRelocaliser> WrappedRelocaliser_Ptr;

  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  /** The number of decisions per fern. */
  int m_decisionsPerFern;

  /** The size of the input depth images. */
  Vector2i m_depthImageSize;

  /** The keyframe harvesting threshold. */
  float m_harvestingThreshold;

  /**
   * The policy deciding whether a frame for which the integrate_rgbd_pose_pair function is called can be actually
   * integrated in the Fern conservatory.
   */
  KeyframeAddPolicy m_keyframeAddPolicy;

  /** The delay before trying to add another keyframe to the Fern conservatory. */
  uint32_t m_keyframeDelay;

  /** The number of ferns to use for relocalisation. */
  int m_numFerns;

  /** The minimum and maximum range of the depth images. */
  Vector2f m_rangeParameters;

  /** The wrapped relocaliser. */
  WrappedRelocaliser_Ptr m_relocaliser;
};

} // namespace itmx

#endif // H_ITMX_FERNRELOCALISER
