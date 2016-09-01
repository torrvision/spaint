/**
 * spaint: AsyncImageSourceEngine.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_ASYNCIMAGESOURCEENGINE
#define H_SPAINT_ASYNCIMAGESOURCEENGINE

#include <InputSource/ImageSourceEngine.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <queue>

namespace spaint
{

/**
 * \brief An instance of this class can be used to wrap an ImageSourceEngine in a separate thread.
 *        The actual image grabbing process is run in a different thread and having an in memory buffer
 *        stores the cached. This allows a latency reduction when processing a sequence stored on disk.
 */
class AsyncImageSourceEngine : public InputSource::ImageSourceEngine
{
  //#################### PRIVATE VARIABLES ####################
private:
  struct RGBDImagePair
  {
    boost::shared_ptr<ITMUChar4Image> rgb;
    boost::shared_ptr<ITMShortImage> rawDepth;
  };

  /** The actual image source. */
  boost::shared_ptr<ImageSourceEngine> m_innerSource;

  /** The thread where the getImages() method of m_innerSource is called. */
  boost::thread m_grabbingThread;

  /** Synchronization mutex. */
  boost::mutex m_bufferMutex;

  /** Condition variables used to handle empty/full buffer events. */
  boost::condition_variable m_bufferFull, m_bufferEmpty;

  /** Queue of cached image pairs. */
  std::queue<RGBDImagePair> m_bufferedImages;

  /** Maximum number of elements allowed in the buffer. */
  size_t m_bufferCapacity;

  /** Image size for the RGB images read by m_innerSource. */
  Vector2i m_rgbImageSize;
  /** Image size for the depth images read by m_innerSource. */
  Vector2i m_depthImageSize;

  /** Set to true when the inner thread must terminate (used in the destructor).*/
  bool m_terminate;

  /** Signals if m_innerSource has more images (other than those already in m_bufferedImages). */
  bool m_hasMoreImages;

public:
  /**
   * \brief Constructor
   * \param innerSource An ImageSourceEngine that will be queried in a separate thread.
   * \param maxBufferSize The maximum number of elements that will be cached. 0 means no limit.
   */
  AsyncImageSourceEngine(ImageSourceEngine *innerSource, size_t maxBufferSize = 0);
  virtual ~AsyncImageSourceEngine();

  virtual ITMLib::ITMRGBDCalib& getCalib();
  virtual Vector2i getDepthImageSize(void);
  virtual void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);
  virtual Vector2i getRGBImageSize(void);
  virtual bool hasMoreImages(void);

private:
  /** Loop executed by the inner thread to grab images from m_innerSource. */
  void grabbingLoop();
};

}

#endif // H_SPAINT_ASYNCIMAGESOURCEENGINE
