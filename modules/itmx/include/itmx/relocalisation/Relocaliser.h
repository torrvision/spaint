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

class Relocaliser
{
  //#################### CONSTRUCTORS ####################
protected:
  Relocaliser() {}

  //#################### DESTRUCTOR ####################
public:
  virtual ~Relocaliser() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  virtual void integrate_rgbd_pose_pair(const ITMUChar4Image *colourImage,
                                        const ITMFloatImage *depthImage,
                                        const Vector4f &depthIntrinsics,
                                        const ORUtils::SE3Pose &cameraPose) = 0;

  virtual boost::optional<ORUtils::SE3Pose> relocalise(const ITMUChar4Image *colourImage,
                                                       const ITMFloatImage *depthImage,
                                                       const Vector4f &depthIntrinsics) = 0;

  virtual void reset() = 0;

  virtual void update() = 0;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<Relocaliser> Relocaliser_Ptr;
typedef boost::shared_ptr<const Relocaliser> Relocaliser_CPtr;

} // namespace itmx

#endif // H_ITMX_RELOCALISER
