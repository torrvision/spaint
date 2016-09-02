/**
 * spaint: AsyncImageSourceEngine.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_ASYNCIMAGESOURCEENGINE
#define H_SPAINT_ASYNCIMAGESOURCEENGINE

#include <queue>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <InputSource/ImageSourceEngine.h>

namespace spaint
{

/**
 * \brief An instance of this class can be used to wrap an ImageSourceEngine in a separate thread.
 *        The actual image grabbing process is run in a different thread and having an in-memory buffer
 *        stores the cached images. This allows a latency reduction when processing a sequence stored on disk.
 */
class AsyncImageSourceEngine : public InputSource::ImageSourceEngine
{
private:

  //#################### NESTED TYPES ####################
  struct RGBDImagePair
  {
    boost::shared_ptr<ITMUChar4Image> rgb;
    boost::shared_ptr<ITMShortImage> rawDepth;
  };

  //#################### PRIVATE VARIABLES ###############
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
  boost::shared_ptr<ImageSourceEngine> m_innerSource;

  /** Image size for the RGB images read by m_innerSource. */
  Vector2i m_rgbImageSize;

  /** A pool where previously used RGBDImagePair are stored to avoid continuously allocating and deallocating them. */
  std::queue<RGBDImagePair> m_rgbdImagePool;

  /** Maximum number of elements stored in m_rgbdImagePool. */
  size_t m_rgbdImagePoolCapacity;

  /** Set to true when the inner thread must terminate (used in the destructor). */
  bool m_terminate;

public:
  //#################### CONSTRUCTORS ###############
  /**
   * \brief Constructs an AsyncImageSourceEngine.
   *        An instance of this class can be used to wrap an ImageSourceEngine in a separate thread.
   *        The actual image grabbing process is run in a different thread and having an in-memory buffer
   *        stores the cached images. This allows a latency reduction when processing a sequence stored on disk.
   *
   * \param innerSource    An ImageSourceEngine that will be queried in a separate thread.
   * \param bufferCapacity The maximum number of elements that will be cached. 0 means no limit.
   */
  AsyncImageSourceEngine(ImageSourceEngine *innerSource, size_t bufferCapacity = 0);

  //#################### DESTRUCTOR #################
  virtual ~AsyncImageSourceEngine();

  //############# PUBLIC MEMBER FUNCTIONS ###########
  virtual ITMLib::ITMRGBDCalib& getCalib();
  virtual Vector2i getDepthImageSize();
  virtual void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);
  virtual Vector2i getRGBImageSize();
  virtual bool hasMoreImages();

private:
  //############# PRIVATE MEMBER FUNCTIONS ##########
  /** Loop executed by the inner thread to grab images from m_innerSource. */
  void grabbingLoop();
};

}

#endif // H_SPAINT_ASYNCIMAGESOURCEENGINE
