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

const Vector2i& MappingClientHandler::get_depth_image_size() const
{
  return m_calib.intrinsics_d.imgSize;
}

MappingClientHandler::RenderingImageHandler_Ptr MappingClientHandler::get_rendering_image()
{
  return RenderingImageHandler_Ptr(new RenderingImageHandler(this));
}

const Vector2i& MappingClientHandler::get_rgb_image_size() const
{
  return m_calib.intrinsics_rgb.imgSize;
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

        RenderingImageHandler_Ptr imageHandler = get_rendering_image();
        if(!imageHandler->get()) break;

        m_dummyFrameMessage->set_frame_index(-1);
        m_dummyFrameMessage->set_rgb_image(imageHandler->get());
        m_frameCompressor->compress_rgbd_frame(*m_dummyFrameMessage, m_headerMessage, *m_frameMessage);

        // TODO: Comment here.
        AckMessage ackMsg;
        m_connectionOk = m_connectionOk && write_message(m_headerMessage) && write_message(*m_frameMessage) && read_message(ackMsg);

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
        std::cout << "Receiving updated rendering pose from client" << std::endl;
#endif

        // Try to read a rendering request message.
        if((m_connectionOk = read_message(renderingRequestMsg)))
        {
          // If that succeeds, store the pose so that it can be picked up by the renderer and send an acknowledgement to the client.
          boost::lock_guard<boost::mutex> lock(m_renderingRequestMutex);
          m_renderingRequestMsg = renderingRequestMsg;
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

    // Construct a dummy frame message to consume messages that cannot be pushed onto the queue
    // and to use as a temporary storage space when sending renderings back to the client.
    m_dummyFrameMessage.reset(new RGBDFrameMessage(rgbImageSize, depthImageSize));

    // Signal to the client that the server is ready.
    m_connectionOk = write_message(AckMessage());
  }
}

}
