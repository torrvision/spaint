/**
 * spaint: AsyncImageSourceEngine.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_ASYNCIMAGESOURCEENGINE
#define H_SPAINT_ASYNCIMAGESOURCEENGINE

#include <queue>

#include <boost/thread.hpp>

#include "../util/ITMImagePtrTypes.h"
#include "../util/ITMObjectPtrTypes.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to read RGB-D images asynchronously from an existing image source.
 *        Images are read from the existing source on a separate thread and stored in an in-memory buffer. This
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
  /** The maximum number of elements allowed in the buffer. */
  size_t m_bufferCapacity;

  /** A queue of cached RGB-D images. */
  std::queue<RGBDImage> m_bufferedImages;

  /** The synchronisation mutex. */
  mutable boost::mutex m_bufferMutex;

  /** A condition variable used to wait for elements to be inserted in the buffer. */
  mutable boost::condition_variable m_bufferNotEmpty;

  /** A condition variable used to wait for elements to be removed from the buffer. */
  boost::condition_variable m_bufferNotFull;

  /** The thread on which images are grabbed from the existing image source. */
  boost::thread m_grabbingThread;

  /** The image source to be decorated. */
  ImageSourceEngine_Ptr m_innerSource;

  /** A pool of reusable RGB-D images. */
  std::queue<RGBDImage> m_rgbdImagePool;

  /** The maximum number of elements that can be stored in the RGB-D image pool. */
  size_t m_rgbdImagePoolCapacity;

  /** A flag set in the destructor to indicate that the grabbing thread should terminate. */
  bool m_terminate;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an asynchronous image source engine.
   *
   * \param innerSource    The image source to be decorated.
   * \param bufferCapacity The maximum number of RGB-D images that will be cached (0 means no limit).
   */
  explicit AsyncImageSourceEngine(ImageSourceEngine *innerSource, size_t bufferCapacity = 0);

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
   * \brief Runs the loop that grabs images from the existing image source.
   */
  void grabbing_loop();
};

}

#endif
