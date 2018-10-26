/**
 * itmx: MappingClientHandler.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "remotemapping/MappingClientHandler.h"

#include <tvgutil/net/AckMessage.h>
using namespace tvgutil;

#ifdef WITH_OPENCV
#include "ocv/OpenCVUtil.h"
#endif

#include "remotemapping/InteractionTypeMessage.h"
#include "remotemapping/RenderingRequestMessage.h"
#include "remotemapping/RGBDCalibrationMessage.h"

//#define DEBUGGING 1

namespace itmx {

//#################### CONSTRUCTORS ####################

MappingClientHandler::MappingClientHandler(int clientID, const boost::shared_ptr<boost::asio::ip::tcp::socket>& sock,
                                           const boost::shared_ptr<const boost::atomic<bool> >& shouldTerminate)
: ClientHandler(clientID, sock, shouldTerminate),
  m_frameMessageQueue(new RGBDFrameMessageQueue(tvgutil::pooled_queue::PES_DISCARD)),
  m_imagesDirty(false),
  m_poseDirty(false)
{
  m_frameMessage.reset(new CompressedRGBDFrameMessage(m_headerMessage));
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

const ITMLib::ITMRGBDCalib& MappingClientHandler::get_calib() const
{
  return m_calib;
}

const Vector2i& MappingClientHandler::get_depth_image_size() const
{
  return m_calib.intrinsics_d.imgSize;
}

const MappingClientHandler::RGBDFrameMessageQueue_Ptr& MappingClientHandler::get_frame_message_queue()
{
  return m_frameMessageQueue;
}

ExclusiveHandle_Ptr<ORUChar4Image_Ptr>::Type MappingClientHandler::get_rendered_image()
{
  return make_exclusive_handle(m_renderedImage, m_renderedImageMutex);
}

ExclusiveHandle_Ptr<boost::optional<RenderingRequestMessage> >::Type MappingClientHandler::get_rendering_request()
{
  return make_exclusive_handle(m_renderingRequestMessage, m_renderingRequestMutex);
}

const Vector2i& MappingClientHandler::get_rgb_image_size() const
{
  return m_calib.intrinsics_rgb.imgSize;
}

const std::string& MappingClientHandler::get_scene_id() const
{
  return m_sceneID;
}

bool MappingClientHandler::images_dirty() const
{
  return m_imagesDirty;
}

bool MappingClientHandler::pose_dirty() const
{
  return m_poseDirty;
}

void MappingClientHandler::run_iter()
{
  InteractionTypeMessage interactionTypeMsg;
  RenderingRequestMessage renderingRequestMsg;

  // First, try to read an interaction type message.
  if((m_connectionOk = read_message(interactionTypeMsg)))
  {
    // If that succeeds, determine the type of interaction the client wants to have with the server and proceed accordingly.
    switch(interactionTypeMsg.extract_value())
    {
      case IT_GETRENDEREDIMAGE:
      {
#if DEBUGGING
        std::cout << "Receiving get rendered image request from client" << std::endl;
#endif

        // Try to grab the rendered image to send across to the client, locking the associated mutex for the duration of the process.
        // If no image has been rendered for the client, early out.
        ExclusiveHandle_Ptr<ORUChar4Image_Ptr>::Type imageHandle = get_rendered_image();
        if(!imageHandle->get())
        {
          std::cerr << "Warning: Client " << m_clientID << " attempted to read a non-existent server-rendered image and is probably deadlocked.\n";
          m_connectionOk = false;
          break;
        }

        // Prepare the rendering response message (we reuse an uncompressed RGB-D frame for this to avoid creating a new message type).
        if(!m_renderingResponseMessage || m_renderingResponseMessage->get_rgb_image_size() != imageHandle->get()->noDims)
        {
          m_renderingResponseMessage.reset(new RGBDFrameMessage(imageHandle->get()->noDims, Vector2i(1,1)));
        }

        m_renderingResponseMessage->set_frame_index(-1);
        m_renderingResponseMessage->set_rgb_image(imageHandle->get());

        // Compress the rendering response message for transmission over the network.
        // FIXME: Consider using a separate frame compressor for rendering responses (to avoid continually resizing this one's internal images).
        m_frameCompressor->compress_rgbd_frame(*m_renderingResponseMessage, m_headerMessage, *m_frameMessage);

        // Send the rendering response to the client, and wait for an acknowledgement before proceeding.
        AckMessage ackMsg;
        m_connectionOk = m_connectionOk && write_message(m_headerMessage) && write_message(*m_frameMessage) && read_message(ackMsg);

        break;
      }
      case IT_HASRENDEREDIMAGE:
      {
        // Send a message to the client indicating whether or not an image has ever been rendered for it, and wait for an acknowledgement before proceeding.
        AckMessage ackMsg;
        m_connectionOk = m_connectionOk && write_message(SimpleMessage<bool>(m_renderedImage.get() != NULL)) && read_message(ackMsg);
        break;
      }
      case IT_SENDFRAME:
      {
#if DEBUGGING
        std::cout << "Receiving frame from client" << std::endl;
#endif

        // Try to read a frame header message.
        if((m_connectionOk = read_message(m_headerMessage)))
        {
          // If that succeeds, set up the frame message accordingly.
          m_frameMessage->set_compressed_image_sizes(m_headerMessage);

          // Now, read the frame message itself.
          if((m_connectionOk = read_message(*m_frameMessage)))
          {
            // If that succeeds, uncompress the images, store them on the frame message queue and send an acknowledgement to the client.
#if DEBUGGING
            std::cout << "Message queue size (" << m_clientID << "): " << m_frameMessageQueue->size() << std::endl;
#endif

            RGBDFrameMessageQueue::PushHandler_Ptr pushHandler = m_frameMessageQueue->begin_push();
            boost::optional<RGBDFrameMessage_Ptr&> elt = pushHandler->get();
            RGBDFrameMessage& msg = elt ? **elt : *m_dummyFrameMessage;
            m_frameCompressor->uncompress_rgbd_frame(*m_frameMessage, msg);

            m_connectionOk = write_message(AckMessage());

#if DEBUGGING
            std::cout << "Got message: " << msg.extract_frame_index() << std::endl;

          #ifdef WITH_OPENCV
            static ORUChar4Image_Ptr rgbImage(new ORUChar4Image(get_rgb_image_size(), true, false));
            msg.extract_rgb_image(rgbImage.get());
            cv::Mat3b cvRGB = OpenCVUtil::make_rgb_image(rgbImage->GetData(MEMORYDEVICE_CPU), rgbImage->noDims.x, rgbImage->noDims.y);
            cv::imshow("RGB", cvRGB);
            cv::waitKey(1);
          #endif
#endif
          }
        }

        break;
      }
      case IT_UPDATERENDERINGREQUEST:
      {
#if DEBUGGING
        std::cout << "Receiving updated rendering request from client" << std::endl;
#endif

        // Try to read a rendering request message.
        if((m_connectionOk = read_message(renderingRequestMsg)))
        {
          // If that succeeds, store the request so that it can be picked up by the renderer, and send an acknowledgement to the client.
          ExclusiveHandle_Ptr<boost::optional<RenderingRequestMessage> >::Type requestHandle = get_rendering_request();
          requestHandle->get() = renderingRequestMsg;
          m_connectionOk = write_message(AckMessage());
        }

        break;
      }
      default:
      {
        break;
      }
    }
  }
}

void MappingClientHandler::run_post()
{
  // Destroy the frame compressor prior to stopping the client handler (this cleanly deallocates CUDA memory and avoids a crash on exit).
  m_frameCompressor.reset();
}

void MappingClientHandler::run_pre()
{
  // Read a calibration message from the client to get its camera's image sizes and calibration parameters.
  RGBDCalibrationMessage calibMsg;
  m_connectionOk = read_message(calibMsg);
#if DEBUGGING
  std::cout << "Received calibration message from client: " << m_clientID << std::endl;
#endif

  // If the calibration message was successfully read:
  if(m_connectionOk)
  {
    // Save the calibration parameters.
    m_calib = calibMsg.extract_calib();

    // Initialise the frame message queue.
    const size_t capacity = 5;
    const Vector2i& rgbImageSize = get_rgb_image_size();
    const Vector2i& depthImageSize = get_depth_image_size();
    m_frameMessageQueue->initialise(capacity, boost::bind(&RGBDFrameMessage::make, rgbImageSize, depthImageSize));

    // Set up the frame compressor.
    m_frameCompressor.reset(new RGBDFrameCompressor(rgbImageSize, depthImageSize, calibMsg.extract_rgb_compression_type(), calibMsg.extract_depth_compression_type()));

    // Construct a dummy frame message to consume messages that cannot be pushed onto the queue.
    m_dummyFrameMessage.reset(new RGBDFrameMessage(rgbImageSize, depthImageSize));

    // Signal to the client that the server is ready.
    m_connectionOk = write_message(AckMessage());
  }
}

void MappingClientHandler::set_images_dirty(bool imagesDirty)
{
  m_imagesDirty = imagesDirty;
}

void MappingClientHandler::set_pose_dirty(bool poseDirty)
{
  m_poseDirty = poseDirty;
}

void MappingClientHandler::set_scene_id(const std::string& sceneID)
{
  m_sceneID = sceneID;
}

}
