/**
 * spaint: AsyncImageSourceEngine.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "imagesources/AsyncImageSourceEngine.h"

#include <stdexcept>

namespace spaint {

//#################### CONSTRUCTORS ####################

AsyncImageSourceEngine::AsyncImageSourceEngine(ImageSourceEngine *innerSource, size_t bufferCapacity)
: m_bufferCapacity(bufferCapacity > 0 ? bufferCapacity : std::numeric_limits<size_t>::max()),
  m_innerSource(innerSource),
  m_terminate(false)
{
  if(!innerSource)
  {
    throw std::runtime_error("Error: Cannot initialise an AsyncImageSourceEngine with a NULL ImageSourceEngine.");
  }

  // Determine the maximum number of RGB-D images to store in the pool.
  const size_t MAX_POOL_CAPACITY = 60;
  m_rgbdImagePoolCapacity = std::min<size_t>(m_bufferCapacity, MAX_POOL_CAPACITY);

  // If the inner source has images available, fill the pool to avoid allocating memory at runtime.
  // If the inner source doesn't have any images available, there is no need to allocate.
  if(m_innerSource->hasMoreImages())
  {
    for(size_t i = 0; i < m_rgbdImagePoolCapacity; ++i)
    {
      RGBDImage rgbdImage;
      rgbdImage.rawDepth.reset(new ITMShortImage(m_innerSource->getDepthImageSize(), true, false));
      rgbdImage.rgb.reset(new ITMUChar4Image(m_innerSource->getRGBImageSize(), true, false));
      m_rgbdImagePool.push(rgbdImage);
    }
  }

  // Start the image grabbing thread.
  m_grabbingThread = boost::thread(boost::bind(&AsyncImageSourceEngine::grabbing_loop, this));
}

//#################### DESTRUCTOR ####################

AsyncImageSourceEngine::~AsyncImageSourceEngine()
{
  // Set the flag that informs the image grabbing thread that it should terminate.
  m_terminate = true;

  // Wake the image grabbing thread (it might be waiting on a full buffer).
  m_bufferNotFull.notify_one();

  // Wait for the image grabbing thread to terminate gracefully.
  m_grabbingThread.join();
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

ITMLib::ITMRGBDCalib AsyncImageSourceEngine::getCalib() const
{
  boost::unique_lock<boost::mutex> lock(m_bufferMutex);

  // If there are images in the buffer, return the first image's calibration; if not, defer to the inner source.
  return !m_bufferedImages.empty() ? m_bufferedImages.front().calib : m_innerSource->getCalib();
}

Vector2i AsyncImageSourceEngine::getDepthImageSize() const
{
  boost::unique_lock<boost::mutex> lock(m_bufferMutex);

  // If there are images in the buffer, return the first image's depth size; if not, defer to the inner source.
  return !m_bufferedImages.empty() ? m_bufferedImages.front().rawDepth->noDims : m_innerSource->getDepthImageSize();
}

void AsyncImageSourceEngine::getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth)
{
  boost::unique_lock<boost::mutex> lock(m_bufferMutex);

  // If there are no more images available, early out.
  if(m_bufferedImages.empty())
  {
    throw std::runtime_error("Error: No more images to get. Make sure to call hasMoreImages before calling getImages.");
  }

  // Otherwise, get the first RGB-D image from the buffer.
  RGBDImage& rgbdImage = m_bufferedImages.front();

  // Ensure that the output images have the correct size (this is generally a no-op).
  rawDepth->ChangeDims(rgbdImage.rawDepth->noDims);
  rgb->ChangeDims(rgbdImage.rgb->noDims);

  // Copy the depth and RGB images from the buffered image into the output images.
  rawDepth->SetFrom(rgbdImage.rawDepth.get(), ITMShortImage::CPU_TO_CPU);
  rgb->SetFrom(rgbdImage.rgb.get(), ITMUChar4Image::CPU_TO_CPU);

  // If there is space available in the RGB-D image pool, store the RGB-D image to avoid reallocating memory later.
  if(m_rgbdImagePool.size() < m_rgbdImagePoolCapacity)
  {
    m_rgbdImagePool.push(rgbdImage);
  }

  // Remove the RGB-D image from the buffer and inform the image grabbing thread that the buffer is not full.
  m_bufferedImages.pop();
  m_bufferNotFull.notify_one();
}

Vector2i AsyncImageSourceEngine::getRGBImageSize() const
{
  boost::unique_lock<boost::mutex> lock(m_bufferMutex);

  // If there are images in the buffer, return the first image's RGB size; if not, defer to the inner source.
  return !m_bufferedImages.empty() ? m_bufferedImages.front().rgb->noDims : m_innerSource->getRGBImageSize();
}

bool AsyncImageSourceEngine::hasMoreImages() const
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
