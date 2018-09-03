/**
 * itmx: BackgroundRelocaliser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_BACKGROUNDRELOCALISER
#define H_ITMX_BACKGROUNDRELOCALISER

#include <boost/atomic.hpp>

#include <orx/base/ORImagePtrTypes.h>
#include <orx/relocalisation/Relocaliser.h>

namespace itmx {

/**
 * \brief An instance of this class can be used to decorate calls to a relocaliser
 *        so that they are performed in the background on a different GPU.
 */
class BackgroundRelocaliser : public orx::Relocaliser
{
  //#################### PRIVATE VARIABLES ####################
private:
  /**
   * An internal image in which to store a copy of a colour image that we are trying to pass to the decorated relocaliser.
   * This internal image will be accessible from the GPU on which relocalisation calls will be performed.
   */
  mutable ORUChar4Image_Ptr m_colourImage;

  /**
   * An internal image in which to store a copy of a depth image that we are trying to pass to the decorated relocaliser.
   * This internal image will be accessible from the GPU on which relocalisation calls will be performed.
   */
  mutable ORFloatImage_Ptr m_depthImage;

  /** The ID of the old GPU on which calls were previously being performed, so that it can be restored later. */
  mutable int m_oldDevice;

  /** The ID of the GPU on which calls to the decorated relocaliser should be performed. */
  int m_relocalisationDevice;

  /** The relocaliser to decorate. */
  orx::Relocaliser_Ptr m_relocaliser;

  /**
   * An atomic flag recording whether or not a relocalisation is currently running. This is used to prevent attempts
   * to train or update the decorated relocaliser during relocalisation, since these would block the main thread and
   * make the application less responsive.
   */
  mutable boost::atomic<bool> m_relocaliserRunning;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a background relocaliser.
   *
   * \param relocaliser           The relocaliser to decorate.
   * \param relocalisationDevice  The ID of the GPU on which calls to the decorated relocaliser should be performed.
   */
  BackgroundRelocaliser(const orx::Relocaliser_Ptr& relocaliser, int relocalisationDevice);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void finish_training();

  /** Override */
  virtual void load_from_disk(const std::string& inputFolder);

  /** Override */
  virtual std::vector<Result> relocalise(const ORUChar4Image *colourImage, const ORFloatImage *depthImage, const Vector4f& depthIntrinsics) const;

  /** Override */
  virtual void reset();

  /** Override */
  virtual void save_to_disk(const std::string& outputFolder) const;

  /** Override */
  virtual void train(const ORUChar4Image *colourImage, const ORFloatImage *depthImage,
                     const Vector4f& depthIntrinsics, const ORUtils::SE3Pose& cameraPose);

  /** Override */
  virtual void update();

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Makes internal copies of the specified colour and depth images that can be accessed from the relocalisation GPU.
   *
   * \note  The input images must be accessible on the CPU at the point at which this function is called.
   *
   * \param colourImage The colour image to copy.
   * \param depthImage  The depth image to copy.
   */
  void copy_images(const ORUChar4Image *colourImage, const ORFloatImage *depthImage) const;

  /**
   * \brief Sets the current GPU to the one on which calls were previously being performed.
   */
  void to_old_gpu() const;

  /**
   * \brief Sets the current GPU to the one on which calls to the decorated relocaliser should be performed.
   */
  void to_relocalisation_gpu() const;
};

}

#endif
