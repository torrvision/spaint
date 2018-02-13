/**
 * itmx: NullRelocaliser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_NULLRELOCALISER
#define H_ITMX_NULLRELOCALISER

#include "Relocaliser.h"

namespace itmx {

/**
 * \brief An instance of this class represents a relocaliser that never tries to relocalise.
 *
 * This can be used to allow the user to reconstruct a scene without relocalisation support.
 */
class NullRelocaliser : public Relocaliser
{
  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void load_from_disk(const std::string& inputFolder);

  /** Override */
  virtual std::vector<Result> relocalise(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage, const Vector4f& depthIntrinsics) const;

  /** Override */
  virtual void reset();

  /** Override */
  virtual void save_to_disk(const std::string& outputFolder) const;

  /** Override */
  virtual void train(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage,
                     const Vector4f& depthIntrinsics, const ORUtils::SE3Pose& cameraPose);
};

}

#endif
