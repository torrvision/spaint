/**
 * spaint: AsyncImageSourceEngine.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_ASYNCIMAGESOURCEENGINE
#define H_SPAINT_ASYNCIMAGESOURCEENGINE

#include <queue>

#include <boost/thread.hpp>

#include <itmx/ITMImagePtrTypes.h>
#include <itmx/ITMObjectPtrTypes.h>

namespace spaint {

/**
 * \brief An instance of this class can be used to read RGB-D images asynchronously from an existing image source.
 *        Images are read from the existing source on a separate thread and stored in an in-memory queue. This
 *        leads to lower latency when processing a disk sequence.
 */
class AsyncImageSourceEngine : public InputSource::ImageSourceEngine
{
  //#################### NESTED TYPES ####################
private:
  /**
   * \brief An instance of this struct can be used to represent an RGB-D image.
   */
  struct RGBDImage
  {
    /** The intrinsic calibration parameters for the camera that produced the RGB-D image. */
    ITMLib::ITMRGBDCalib calib;

    /** The depth component of the RGB-D image. */
    ITMShortImage_Ptr rawDepth;

    /** The RGB component of the RGB-D image. */
    ITMUChar4Image_Ptr rgb;
  };

  //#################### PRIVATE VARIABLES ####################
private:
  /** The thread on which images are grabbed from the existing image source. */
  boost::thread m_grabber;

  /** A flag set in the destructor to indicate that the image grabber should terminate. */
  bool m_grabberShouldTerminate;

  /** The image source from which to obtain the images to cache. */
  ImageSourceEngine_Ptr m_innerSource;

  /** The synchronisation mutex. */
  mutable boost::mutex m_mutex;

  /** A pool of reusable RGB-D images. */
  std::queue<RGBDImage> m_pool;

  /** The maximum number of elements that can be stored in the RGB-D image pool. */
  size_t m_poolCapacity;

  /** A queue in which to cache images from the inner source. */
  std::queue<RGBDImage> m_queue;

  /** The maximum number of images to cache. */
  size_t m_queueCapacity;

  /** A condition variable used to wait for elements to be added to the queue. */
  mutable boost::condition_variable m_queueNotEmpty;

  /** A condition variable used to wait for elements to be removed from the queue. */
  boost::condition_variable m_queueNotFull;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an asynchronous image source engine.
   *
   * \param innerSource   The image source from which to obtain the images to cache.
   * \param queueCapacity The maximum number of images to cache (0 means no limit).
   */
  explicit AsyncImageSourceEngine(ImageSourceEngine *innerSource, size_t queueCapacity = 0);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the image source engine.
   */
  virtual ~AsyncImageSourceEngine();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual ITMLib::ITMRGBDCalib getCalib() const;

  /** Override */
  virtual Vector2i getDepthImageSize() const;

  /** Override */
  virtual void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);

  /** Override */
  virtual Vector2i getRGBImageSize() const;

  /** Override */
  virtual bool hasMoreImages() const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Runs the image grabber.
   */
  void run_image_grabber();
};

}

#endif
