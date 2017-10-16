/**
 * itmx: CompressedRGBDFrameMessage.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_COMPRESSEDRGBDFRAMEMESSAGE
#define H_ITMX_COMPRESSEDRGBDFRAMEMESSAGE

#include <ORUtils/SE3Pose.h>

#include "MappingMessage.h"
#include "CompressedRGBDFrameHeaderMessage.h"
#include "../base/ITMImagePtrTypes.h"

namespace itmx {

/**
 * \brief An instance of this class represents a message containing a single frame of compressed RGB-D data (frame index + pose + RGB-D).
 */
class CompressedRGBDFrameMessage : public MappingMessage
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The byte segment within the message data that corresponds to the compressed depth image. */
  Segment m_depthImageSegment;

  /** The byte segment within the message data that corresponds to the frame index. */
  Segment m_frameIndexSegment;

  /** The byte segment within the message data that corresponds to the pose. */
  Segment m_poseSegment;

  /** The byte segment within the message data that corresponds to the compressed RGB image. */
  Segment m_rgbImageSegment;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a compressed RGB-D frame message.
   *
   * \param messageHeader  The header of the compressed message, specifying the size of the depth and colour segments.
   */
  CompressedRGBDFrameMessage(const CompressedRGBDFrameHeaderMessage& messageHeader);

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Makes a compressed RGB-D frame message.
   *
   * \param messageHeader The header of the compressed message, specifying the size of the depth and colour segments.
   */
  static boost::shared_ptr<CompressedRGBDFrameMessage> make(const CompressedRGBDFrameHeaderMessage& messageHeader);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Extracts the compressed depth image data from the message and writes it into the specified destination vector.
   *
   * \param depthImageData  The vector into which to write the message's compressed depth image. It will be resized as necessary.
   */
  void extract_depth_image_data(std::vector<uint8_t>& depthImageData) const;

  /**
   * \brief Extracts the frame index from the message.
   *
   * \return  The message's frame index.
   */
  int extract_frame_index() const;

  /**
   * \brief Extracts the pose from the message.
   *
   * \return  The message's pose.
   */
  ORUtils::SE3Pose extract_pose() const;

  /**
   * \brief Extracts the compressed colour image data from the message and writes it into the specified destination vector.
   *
   * \param rgbImageData  The vector into which to write the message's compressed colour image. It will be resized as necessary.
   */
  void extract_rgb_image_data(std::vector<uint8_t>& rgbImageData) const;

  /**
   * \brief Copies a compressed depth image into the appropriate byte segment in the message.
   *
   * \param depthImageData  The compressed depth image data.
   */
  void set_depth_image_data(const std::vector<uint8_t>& depthImageData);

  /**
   * \brief Copies a frame index into the appropriate byte segment in the message.
   *
   * \param frameIndex  The frame index.
   */
  void set_frame_index(int frameIndex);

  /**
   * \brief Copies a pose into the appropriate byte segment in the message.
   *
   * \param pose  The pose.
   */
  void set_pose(const ORUtils::SE3Pose& pose);

  /**
   * \brief Sets the segment sizes for the depth and colour images according to the compressed message header. Resizes the raw data storage accordingly.
   *
   * \param messageHeader  The header of the compressed message, specifying the size of the depth and colour segments.
   */
  void set_compressed_image_sizes(const CompressedRGBDFrameHeaderMessage& messageHeader);

  /**
   * \brief Copies a compressed RGB image into the appropriate byte segment in the message.
   *
   * \param rgbImage  The compressed RGB image data.
   */
  void set_rgb_image_data(const std::vector<uint8_t>& rgbImageData);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<CompressedRGBDFrameMessage> CompressedRGBDFrameMessage_Ptr;
typedef boost::shared_ptr<const CompressedRGBDFrameMessage> CompressedRGBDFrameMessage_CPtr;

}

#endif
