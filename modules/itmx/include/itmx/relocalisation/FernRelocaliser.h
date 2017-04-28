/**
 * itmx: FernRelocaliser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_FERNRELOCALISER
#define H_ITMX_FERNRELOCALISER

#include "Relocaliser.h"

#include <FernRelocLib/Relocaliser.h>

namespace itmx {

class FernRelocaliser : public Relocaliser
{
  //#################### ENUMS ####################
public:
  enum KeyframeAddPolicy { ALWAYS_TRY_ADD, DELAY_AFTER_RELOCALISATION };

  //#################### CONSTRUCTORS ####################
protected:
  FernRelocaliser(Vector2i depthImageSize,
                  float viewFrustumMin,
                  float viewFrustumMax,
                  float harvestingThreshold,
                  int numFerns,
                  int decisionsPerFern,
                  KeyframeAddPolicy keyframeAddPolicy = DELAY_AFTER_RELOCALISATION);

  //#################### PUBLIC VIRTUAL MEMBER FUNCTIONS ####################
public:
  virtual void integrate_rgbd_pose_pair(const ITMUChar4Image *colourImage,
                                        const ITMFloatImage *depthImage,
                                        const Vector4f &depthIntrinsics,
                                        const ORUtils::SE3Pose &cameraPose);

  virtual boost::optional<ORUtils::SE3Pose>
      relocalise(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage, const Vector4f &depthIntrinsics);

  virtual void reset();

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
