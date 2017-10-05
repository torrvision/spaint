/**
 * itmx: MappingMessage.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_MAPPINGMESSAGE
#define H_ITMX_MAPPINGMESSAGE

#include <vector>

#include <ORUtils/SE3Pose.h>

#include "../base/ITMImagePtrTypes.h"

namespace itmx {

/**
 * \brief An instance of this class represents a message containing a single frame of mapping data (frame index + pose + RGB-D).
 */
class MappingMessage
{
  //#################### TYPEDEFS ####################
private:
  /** An (offset, size) pair used to specify a byte segment within the message data. */
  typedef std::pair<size_t,size_t> Segment;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The message data. */
  std::vector<char> m_data;

  /** The byte segment within the message data that corresponds to the depth image. */
  Segment m_depthImageSegment;

  /** The byte segment within the message data that corresponds to the frame index. */
  Segment m_frameIndexSegment;

  /** The byte segment within the message data that corresponds to the pose. */
  Segment m_poseSegment;

  /** The byte segment within the message data that corresponds to the RGB image. */
  Segment m_rgbImageSegment;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a mapping message.
   *
   * \param rgbImageSize    The size of the frame's RGB image.
   * \param depthImageSize  The size of the frame's depth image.
   */
  MappingMessage(const Vector2i& rgbImageSize, const Vector2i& depthImageSize);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets a raw pointer to the message data.
   *
   * \return  A raw pointer to the message data.
   */
  char *get_data_ptr();

  /**
   * \brief Gets a raw pointer to the message data.
   *
   * \return  A raw pointer to the message data.
   */
  const char *get_data_ptr() const;

  /**
   * \brief Gets the size of the message.
   *
   * \return  The size of the message.
   */
  size_t get_size() const;

  /**
   * \brief Extracts the depth image from the message and writes it into the specified destination image.
   *
   * \param depthImage  The destination image into which to write the message's depth image.
   */
  void extract_depth_image(ITMShortImage *depthImage) const;

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
   * \brief Extracts the RGB image from the message and writes it into the specified destination image.
   *
   * \param rgbImage  The destination image into which to write the message's RGB image.
   */
  void extract_rgb_image(ITMUChar4Image *rgbImage) const;

  /**
   * \brief Copies a depth image into the appropriate byte segment in the message.
   *
   * \param depthImage  The depth image.
   */
  void set_depth_image(const ITMShortImage_CPtr& depthImage);

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
   * \brief Copies an RGB image into the appropriate byte segment in the message.
   *
   * \param rgbImage  The RGB image.
   */
  void set_rgb_image(const ITMUChar4Image_CPtr& rgbImage);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<MappingMessage> MappingMessage_Ptr;
typedef boost::shared_ptr<const MappingMessage> MappingMessage_CPtr;

}

#endif
