/**
 * spaint: AsyncImageSourceEngine.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_ASYNCIMAGESOURCEENGINE
#define H_SPAINT_ASYNCIMAGESOURCEENGINE

#include <queue>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "../util/ITMImagePtrTypes.h"
#include "../util/ITMObjectPtrTypes.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to wrap an ImageSourceEngine in a separate thread.
 *        The actual image grabbing process is run in a different thread and having an in-memory buffer
 *        stores the cached images. This allows a latency reduction when processing a sequence stored on disk.
 */
class AsyncImageSourceEngine : public InputSource::ImageSourceEngine
{
  //#################### NESTED TYPES ####################
private:
  struct RGBDImagePair
  {
    ITMShortImage_Ptr rawDepth;
    ITMUChar4Image_Ptr rgb;
  };

  //#################### PRIVATE VARIABLES ####################
private:
  /** Maximum number of elements allowed in the buffer. */
  size_t m_bufferCapacity;

  /** Queue of cached image pairs. */
  std::queue<RGBDImagePair> m_bufferedImages;

  /** Synchronization mutex. */
  boost::mutex m_bufferMutex;

  /** Condition variable used to wait for elements to be inserted in the buffer. */
  boost::condition_variable m_bufferNotEmpty;

  /** Condition variable used to wait for elements to be removed from the buffer. */
  boost::condition_variable m_bufferNotFull;

  /** Image size for the depth images read by m_innerSource. */
  Vector2i m_depthImageSize;

  /** The thread where the getImages() method of m_innerSource is called. */
  boost::thread m_grabbingThread;

  /** Signals if m_innerSource has more images (other than those already in m_bufferedImages).
   *  Stores the result of the most recent call to m_innerSource->hasMoreImages().
   */
  bool m_hasMoreImages;

  /** The actual image source. */
  ImageSourceEngine_Ptr m_innerSource;

  /** Image size for the RGB images read by m_innerSource. */
  Vector2i m_rgbImageSize;

  /** A pool where previously used RGBDImagePair are stored to avoid continuously allocating and deallocating them. */
  std::queue<RGBDImagePair> m_rgbdImagePool;

  /** Maximum number of elements stored in m_rgbdImagePool. */
  size_t m_rgbdImagePoolCapacity;

  /** Set to true when the inner thread must terminate (used in the destructor). */
  bool m_terminate;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an AsyncImageSourceEngine.
   *        An instance of this class can be used to wrap an ImageSourceEngine in a separate thread.
   *        The actual image grabbing process is run in a different thread and having an in-memory buffer
   *        stores the cached images. This allows a latency reduction when processing a sequence stored on disk.
   *
   * \param innerSource    An ImageSourceEngine that will be queried in a separate thread.
   * \param bufferCapacity The maximum number of elements that will be cached. 0 means no limit.
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
  virtual ITMLib::ITMRGBDCalib& getCalib();

  /** Override */
  virtual Vector2i getDepthImageSize();

  /** Override */
  virtual void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);

  /** Override */
  virtual Vector2i getRGBImageSize();

  /** Override */
  virtual bool hasMoreImages();

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /** Loop executed by the inner thread to grab images from m_innerSource. */
  void grabbing_loop();
};

}

#endif
