/**
 * itmx: BackgroundRelocaliser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_BACKGROUNDRELOCALISER
#define H_ITMX_BACKGROUNDRELOCALISER

#include <boost/atomic.hpp>

#include "Relocaliser.h"
#include "../base/ITMImagePtrTypes.h"

namespace itmx {

/**
 * \brief An instance of this class can be used to decorate calls to a relocaliser
 *        so that they are performed in the background on a different GPU.
 */
class BackgroundRelocaliser : public Relocaliser
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** TODO */
  mutable ITMUChar4Image_Ptr m_colourImage;

  /** TODO */
  mutable ITMFloatImage_Ptr m_depthImage;

  /** The ID of the old GPU on which calls were previously being performed, so that it can be restored later. */
  mutable int m_oldDevice;

  /** The ID of the GPU on which relocalisation calls should be performed. */
  int m_relocalisationDevice;

  /** The relocaliser to decorate. */
  Relocaliser_Ptr m_relocaliser;

  /** TODO */
  mutable boost::atomic<bool> m_relocaliserRunning;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief TODO
   */
  BackgroundRelocaliser(const Relocaliser_Ptr& relocaliser, int relocalisationDevice);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual boost::optional<Result> relocalise(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage, const Vector4f& depthIntrinsics) const;

  /** Override */
  virtual void reset();

  /** Override */
  virtual void train(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage,
                     const Vector4f& depthIntrinsics, const ORUtils::SE3Pose& cameraPose);

  /** Override */
  virtual void update();

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief TODO
   */
  void copy_images(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage) const;

  /**
   * \brief TODO
   */
  void to_old_gpu() const;

  /**
   * \brief TODO
   */
  void to_relocalisation_gpu() const;
};

}

#endif
