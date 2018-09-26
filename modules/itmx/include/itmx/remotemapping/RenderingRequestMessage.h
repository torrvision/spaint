/**
 * itmx: RenderingRequestMessage.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_ITMX_RENDERINGREQUESTMESSAGE
#define H_ITMX_RENDERINGREQUESTMESSAGE

#include <ORUtils/Math.h>

#include "MappingMessage.h"

namespace itmx {

/**
 * \brief An instance of this class represents a message containing a request from a mapping client for the server to render a visualisation of the scene.
 */
class RenderingRequestMessage : public MappingMessage
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The byte segment within the message data that corresponds to the size of image to render. */
  Segment m_imageSizeSegment;

  /** The byte segment within the message data that corresponds to the pose from which to render. */
  Segment m_poseSegment;

  /** The byte segment within the message that corresponds to the type of visualisation to render. */
  Segment m_visualisationTypeSegment;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a rendering request message.
   */
  RenderingRequestMessage();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Extracts the size of image to render from the message.
   *
   * \return  The size of image to render.
   */
  Vector2i extract_image_size() const;

  /**
   * \brief Extracts the pose from which to render from the message.
   *
   * \return  The pose from which to render.
   */
  ORUtils::SE3Pose extract_pose() const;

  /**
   * \brief Extracts the type of visualisation to render from the message.
   *
   * \return  The type of visualisation to render.
   */
  int extract_visualisation_type() const;

  /**
   * \brief Sets the size of image to render.
   *
   * \param imgSize The size of image to render.
   */
  void set_image_size(const Vector2i& imgSize);

  /**
   * \brief Sets the pose from which to render.
   *
   * \param pose  The pose from which to render.
   */
  void set_pose(const ORUtils::SE3Pose& pose);

  /**
   * \brief Sets the type of visualisation to render.
   *
   * \param visualisationType The type of visualisation to render.
   */
  void set_visualisation_type(int visualisationType);
};

}

#endif
