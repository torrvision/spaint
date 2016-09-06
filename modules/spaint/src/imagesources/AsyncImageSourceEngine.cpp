/**
 * spaint: AsyncImageSourceEngine.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "imagesources/AsyncImageSourceEngine.h"

#include <stdexcept>

namespace spaint {

//#################### CONSTRUCTORS ####################

AsyncImageSourceEngine::AsyncImageSourceEngine(ImageSourceEngine *innerSource, size_t queueCapacity)
: m_grabberShouldTerminate(false),
  m_innerSource(innerSource),
  m_queueCapacity(queueCapacity > 0 ? queueCapacity : std::numeric_limits<size_t>::max())
{
  if(!innerSource)
  {
    throw std::runtime_error("Error: Cannot initialise an AsyncImageSourceEngine with a NULL ImageSourceEngine.");
  }

  // Determine the maximum number of RGB-D images to store in the pool.
  const size_t MAX_POOL_CAPACITY = 60;
  m_poolCapacity = std::min(m_queueCapacity, MAX_POOL_CAPACITY);

  // If the inner source has images available, fill the pool to avoid allocating memory at runtime.
  // If the inner source doesn't have any images available, there is no need to allocate.
  if(m_innerSource->hasMoreImages())
  {
    for(size_t i = 0; i < m_poolCapacity; ++i)
    {
      RGBDImage rgbdImage;
      rgbdImage.rawDepth.reset(new ITMShortImage(m_innerSource->getDepthImageSize(), true, false));
      rgbdImage.rgb.reset(new ITMUChar4Image(m_innerSource->getRGBImageSize(), true, false));
      m_pool.push(rgbdImage);
    }
  }

  // Start the image grabber.
  m_grabber = boost::thread(boost::bind(&AsyncImageSourceEngine::run_image_grabber, this));
}

//#################### DESTRUCTOR ####################

AsyncImageSourceEngine::~AsyncImageSourceEngine()
{
  // Set the flag that informs the image grabber that it should terminate.
  m_grabberShouldTerminate = true;

  // Wake the image grabber (it might be waiting on a full queue).
  m_queueNotFull.notify_one();

  // Wait for the image grabber to terminate gracefully.
  m_grabber.join();
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

ITMLib::ITMRGBDCalib AsyncImageSourceEngine::getCalib() const
{
  boost::unique_lock<boost::mutex> lock(m_mutex);

  // If there are images in the queue, return the first image's calibration; if not, defer to the inner source.
  return !m_queue.empty() ? m_queue.front().calib : m_innerSource->getCalib();
}

Vector2i AsyncImageSourceEngine::getDepthImageSize() const
{
  boost::unique_lock<boost::mutex> lock(m_mutex);

  // If there are images in the queue, return the first image's depth size; if not, defer to the inner source.
  return !m_queue.empty() ? m_queue.front().rawDepth->noDims : m_innerSource->getDepthImageSize();
}

void AsyncImageSourceEngine::getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth)
{
  boost::unique_lock<boost::mutex> lock(m_mutex);

  // If there are no more images available, early out.
  if(m_queue.empty())
  {
    throw std::runtime_error("Error: No more images to get. Make sure to call hasMoreImages before calling getImages.");
  }

  // Otherwise, get the first RGB-D image from the queue.
  RGBDImage& rgbdImage = m_queue.front();

  // Ensure that the output images have the correct size (this is generally a no-op).
  rawDepth->ChangeDims(rgbdImage.rawDepth->noDims);
  rgb->ChangeDims(rgbdImage.rgb->noDims);

  // Copy the depth and RGB images from the queued image into the output images.
  rawDepth->SetFrom(rgbdImage.rawDepth.get(), ITMShortImage::CPU_TO_CPU);
  rgb->SetFrom(rgbdImage.rgb.get(), ITMUChar4Image::CPU_TO_CPU);

  // If there is space available in the RGB-D image pool, store the RGB-D image to avoid reallocating memory later.
  if(m_pool.size() < m_poolCapacity)
  {
    m_pool.push(rgbdImage);
  }

  // Remove the RGB-D image from the queue and inform the image grabber that the queue is not full.
  m_queue.pop();
  m_queueNotFull.notify_one();
}

Vector2i AsyncImageSourceEngine::getRGBImageSize() const
{
  boost::unique_lock<boost::mutex> lock(m_mutex);

  // If there are images in the queue, return the first image's RGB size; if not, defer to the inner source.
  return !m_queue.empty() ? m_queue.front().rgb->noDims : m_innerSource->getRGBImageSize();
}

bool AsyncImageSourceEngine::hasMoreImages() const
{
  // We need to grab the mutex in case the queue is empty, in which case we need to wait to see if an image becomes available.
  boost::unique_lock<boost::mutex> lock(m_mutex);

  // If the inner source has more images, wait for one to be added to the queue by the image grabber.
  while(m_innerSource->hasMoreImages() && m_queue.empty()) m_queueNotEmpty.wait(lock);

  // At this point, either there is now an image in the queue, in which case we return true,
  // or the inner source has terminated, in which case we return false.
  return !m_queue.empty();
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void AsyncImageSourceEngine::run_image_grabber()
{
  while(!m_grabberShouldTerminate)
  {
    boost::unique_lock<boost::mutex> lock(m_mutex);

    // If the queue is full, wait until some images have been consumed or termination is requested.
    while(!m_grabberShouldTerminate && m_queue.size() >= m_queueCapacity) m_queueNotFull.wait(lock);

    // If we were asked to terminate, do so.
    if(m_grabberShouldTerminate) return;

    // If there are no more images available from the inner source, notify anyone waiting for an image and terminate.
    if(!m_innerSource->hasMoreImages())
    {
      m_queueNotEmpty.notify_one();
      return;
    }

    // Construct an RGB-D image into which to copy the data from the inner source.
    RGBDImage rgbdImage;
    if(!m_pool.empty())
    {
      // If possible, reuse an existing RGB-D image from the pool rather than allocating new memory.
      rgbdImage = m_pool.front();
      m_pool.pop();

      // Ensure that the depth and RGB images have the correct size (this is a no-op unless the size of
      // the images produced by the inner source has changed since we put the RGB-D image in the pool).
      rgbdImage.rawDepth->ChangeDims(m_innerSource->getDepthImageSize());
      rgbdImage.rgb->ChangeDims(m_innerSource->getRGBImageSize());
    }
    else
    {
      // If there was no existing image available from the pool, allocate new memory for the RGB-D image.
      rgbdImage.rawDepth.reset(new ITMShortImage(m_innerSource->getDepthImageSize(), true, false));
      rgbdImage.rgb.reset(new ITMUChar4Image(m_innerSource->getRGBImageSize(), true, false));
    }

    // Get the calibration for the RGB-D image from the inner source.
    rgbdImage.calib = m_innerSource->getCalib();

    // Copy the images from the inner source into the RGB-D image.
    m_innerSource->getImages(rgbdImage.rgb.get(), rgbdImage.rawDepth.get());

    // Add the RGB-D image to the queue and inform the main thread that images are available.
    m_queue.push(rgbdImage);
    m_queueNotEmpty.notify_one();
  }
}

}
