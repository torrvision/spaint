/**
 * spaint: AsyncImageSourceEngine.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include <util/AsyncImageSourceEngine.h>

#include <stdexcept>

namespace spaint
{

AsyncImageSourceEngine::AsyncImageSourceEngine(ImageSourceEngine *innerSource, size_t bufferCapacity)
{
  if (!innerSource)
    throw std::runtime_error("Cannot initialise an AsyncImageSourceEngine with a NULL ImageSourceEngine.");

  m_innerSource.reset(innerSource);
  m_bufferCapacity = bufferCapacity > 0 ? bufferCapacity : std::numeric_limits<size_t>::max();

  m_rgbImageSize = m_innerSource->getRGBImageSize();
  m_depthImageSize = m_innerSource->getDepthImageSize();

  m_rgbdImagePoolCapacity = std::min<size_t>(m_bufferCapacity, 60);
  // Fill the pool to avoid allocating memory at runtime (assuming m_bufferCapacity <= m_rgbdImagePoolCapacity).
  for (size_t i = 0; i < m_rgbdImagePoolCapacity; ++i)
  {
    RGBDImagePair pair;
    pair.rgb.reset(new ITMUChar4Image(m_rgbImageSize, true, false));
    pair.rawDepth.reset(new ITMShortImage(m_depthImageSize, true, false));

    m_rgbdImagePool.push(pair);
  }

  m_terminate = false;
  m_hasMoreImages = m_innerSource->hasMoreImages();

  m_grabbingThread = boost::thread(boost::bind(&AsyncImageSourceEngine::grabbingLoop, this));
}

AsyncImageSourceEngine::~AsyncImageSourceEngine()
{
  // Signal the thread that we are done
  m_terminate = true;
  m_bufferNotFull.notify_one(); // To awake the thread if the buffer was full

  // Wait for it...
  m_grabbingThread.join();
}

void AsyncImageSourceEngine::grabbingLoop()
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

    m_hasMoreImages = m_innerSource->hasMoreImages();
    if (!m_hasMoreImages)
    {
      m_bufferNotEmpty.notify_one(); // Wake up waiting thread
      return;
    }

    RGBDImagePair newImages;
    if (!m_rgbdImagePool.empty())
    {
      // If m_rgbdImagePool contains a preallocated pair use that instead of allocating new memory
      newImages = m_rgbdImagePool.front();
      m_rgbdImagePool.pop();
    }
    else
    {
      // Perform CPU-only allocation
      newImages.rgb.reset(new ITMUChar4Image(m_rgbImageSize, true, false));
      newImages.rawDepth.reset(new ITMShortImage(m_depthImageSize, true, false));
    }

    // Get image data
    m_innerSource->getImages(newImages.rgb.get(), newImages.rawDepth.get());

    // Put the images into the queue
    m_bufferedImages.push(newImages);

    // Notify the main thread
    m_bufferNotEmpty.notify_one();
  }
}

ITMLib::ITMRGBDCalib& AsyncImageSourceEngine::getCalib()
{
  return m_innerSource->getCalib();
}

bool AsyncImageSourceEngine::hasMoreImages()
{
  // Need to grab the mutex in case the buffer is empty, since then we will need to wait to see if one image is coming
  boost::unique_lock<boost::mutex> lock(m_bufferMutex);

  while (m_hasMoreImages && m_bufferedImages.empty()) m_bufferNotEmpty.wait(lock);

  // If the buffer is empty it means that the other thread finished its work
  if (m_bufferedImages.empty() && m_hasMoreImages) throw std::runtime_error("Synchronization error.");

  return !m_bufferedImages.empty();
}

Vector2i AsyncImageSourceEngine::getRGBImageSize()
{
  return m_rgbImageSize;
}

Vector2i AsyncImageSourceEngine::getDepthImageSize()
{
  return m_depthImageSize;
}

void AsyncImageSourceEngine::getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth)
{
  boost::unique_lock<boost::mutex> lock(m_bufferMutex);

  // Wait until an image pair is ready
  while (m_hasMoreImages && m_bufferedImages.empty()) m_bufferNotEmpty.wait(lock);

  // getImages should not be called if hasMoreImages returned false.
  if (m_bufferedImages.empty()) throw std::runtime_error("No more images to get. Need to call hasMoreImages() before getImages()");

  RGBDImagePair &imagePair = m_bufferedImages.front();

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
}
