/**
 * spaint: AsyncImageSourceEngine.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "imagesources/AsyncImageSourceEngine.h"

#include <stdexcept>

namespace spaint {

//#################### CONSTRUCTORS ####################

AsyncImageSourceEngine::AsyncImageSourceEngine(ImageSourceEngine *innerSource, size_t bufferCapacity)
{
  // Never keep more than this number of pairs allocated in the memory pool.
  static const size_t MAX_MEMORY_POOL_CAPACITY = 60;

  if (!innerSource)
    throw std::runtime_error("Cannot initialise an AsyncImageSourceEngine with a NULL ImageSourceEngine.");

  m_innerSource.reset(innerSource);
  m_bufferCapacity = bufferCapacity > 0 ? bufferCapacity : std::numeric_limits<size_t>::max();

  m_rgbdImagePoolCapacity = std::min<size_t>(m_bufferCapacity, MAX_MEMORY_POOL_CAPACITY);

  if (m_innerSource->hasMoreImages())
  {
    // Fill the pool to avoid allocating memory at runtime (assuming m_bufferCapacity <= m_rgbdImagePoolCapacity).
    // Only do it if the inner source actually has images available. If not then there is no need to allocate.
    for (size_t i = 0; i < m_rgbdImagePoolCapacity; ++i)
    {
      RGBDImage pair;
      pair.rgb.reset(new ITMUChar4Image(m_innerSource->getRGBImageSize(), true, false));
      pair.rawDepth.reset(new ITMShortImage(m_innerSource->getDepthImageSize(), true, false));

      m_rgbdImagePool.push(pair);
    }
  }

  m_terminate = false;

  m_grabbingThread = boost::thread(boost::bind(&AsyncImageSourceEngine::grabbing_loop, this));
}

//#################### DESTRUCTOR ####################

AsyncImageSourceEngine::~AsyncImageSourceEngine()
{
  // Signal the thread that we are done
  m_terminate = true;
  m_bufferNotFull.notify_one(); // To awake the thread if the buffer was full

  // Wait for it...
  m_grabbingThread.join();
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

ITMLib::ITMRGBDCalib& AsyncImageSourceEngine::getCalib()
{
  boost::unique_lock<boost::mutex> lock(m_bufferMutex);

  // If there are images in the buffer return the calib from the first one, otherwise defer to decorated image source.
  return !m_bufferedImages.empty() ? m_bufferedImages.front().calib : m_innerSource->getCalib();
}

Vector2i AsyncImageSourceEngine::getDepthImageSize()
{
  boost::unique_lock<boost::mutex> lock(m_bufferMutex);

  // If there are images in the buffer return the depth size of the first one, otherwise defer to decorated image source.
  return !m_bufferedImages.empty() ? m_bufferedImages.front().rawDepth->noDims : m_innerSource->getDepthImageSize();
}

void AsyncImageSourceEngine::getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth)
{
  boost::unique_lock<boost::mutex> lock(m_bufferMutex);

  // getImages should not be called if hasMoreImages returned false.
  if (m_bufferedImages.empty()) throw std::runtime_error("No more images to get. Need to call hasMoreImages() before getImages()");

  RGBDImage &imagePair = m_bufferedImages.front();

  // Resize output images (no-op in the typical case)
  rgb->ChangeDims(imagePair.rgb->noDims);
  rawDepth->ChangeDims(imagePair.rawDepth->noDims);

  // Copy images
  rgb->SetFrom(imagePair.rgb.get(), ITMUChar4Image::CPU_TO_CPU);
  rawDepth->SetFrom(imagePair.rawDepth.get(), ITMShortImage::CPU_TO_CPU);

  // If there is space available in m_rgbdImagePool, store the imagePair to avoid reallocating memory later.
  if (m_rgbdImagePool.size() < m_rgbdImagePoolCapacity)
  {
    m_rgbdImagePool.push(imagePair);
  }

  m_bufferedImages.pop(); // Remove element from the buffer.
  // Notify producer that the buffer has one less element.
  m_bufferNotFull.notify_one();
}

Vector2i AsyncImageSourceEngine::getRGBImageSize()
{
  boost::unique_lock<boost::mutex> lock(m_bufferMutex);

  // If there are images in the buffer return the rgb size of the first one, otherwise defer to decorated image source.
  return !m_bufferedImages.empty() ? m_bufferedImages.front().rgb->noDims : m_innerSource->getRGBImageSize();
}

bool AsyncImageSourceEngine::hasMoreImages()
{
  // Need to grab the mutex in case the buffer is empty, since then we will need to wait to see if one image is coming
  boost::unique_lock<boost::mutex> lock(m_bufferMutex);

  // If the inner source has more images wait for one to be put in the buffer by the grabbing loop.
  while (m_innerSource->hasMoreImages() && m_bufferedImages.empty()) m_bufferNotEmpty.wait(lock);

  // If the buffer is empty when we reach this point it means that the other thread finished its work
  if (m_bufferedImages.empty() && m_innerSource->hasMoreImages())
    throw std::runtime_error("Synchronization error.");

  return !m_bufferedImages.empty();
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void AsyncImageSourceEngine::grabbing_loop()
{
  while (!m_terminate)
  {
    // Grab the mutex
    boost::unique_lock<boost::mutex> lock(m_bufferMutex);

    // If the buffer is full wait until some images have been consumed or termination is requested.
    while (!m_terminate && m_bufferedImages.size() >= m_bufferCapacity) m_bufferNotFull.wait(lock);

    // If we need to terminate, return.
    if (m_terminate)
    {
      return;
    }

    if (!m_innerSource->hasMoreImages())
    {
      m_bufferNotEmpty.notify_one(); // Wake up waiting thread
      return;
    }

    RGBDImage newImages;
    if (!m_rgbdImagePool.empty())
    {
      // If m_rgbdImagePool contains a preallocated pair use that instead of allocating new memory
      newImages = m_rgbdImagePool.front();
      m_rgbdImagePool.pop();
    }
    else
    {
      // Perform CPU-only allocation
      newImages.rgb.reset(new ITMUChar4Image(m_innerSource->getRGBImageSize(), true, false));
      newImages.rawDepth.reset(new ITMShortImage(m_innerSource->getDepthImageSize(), true, false));
    }

    // Get calibration from the inner engine.
    newImages.calib = m_innerSource->getCalib();

    // Resize output images if size is different (no-op if the size in the inner engine has not changed or
    // the images were allocated in the else above).
    newImages.rgb->ChangeDims(m_innerSource->getRGBImageSize());
    newImages.rawDepth->ChangeDims(m_innerSource->getDepthImageSize());

    // Get the images
    m_innerSource->getImages(newImages.rgb.get(), newImages.rawDepth.get());

    // Put the images into the queue
    m_bufferedImages.push(newImages);

    // Notify the main thread
    m_bufferNotEmpty.notify_one();
  }
}

}
