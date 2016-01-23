/**
 * spaintgui: Subwindow.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINTGUI_SUBWINDOW
#define H_SPAINTGUI_SUBWINDOW

#include "../core/Raycaster.h"

/**
  * \brief An instance of this class can be used to represent a sub-window into
  *        which different types of scene visualisation can be rendered.
  */
class Subwindow
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<ITMUChar4Image> ITMUChar4Image_Ptr;
  typedef boost::shared_ptr<const ITMUChar4Image> ITMUChar4Image_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The location of the bottom-right of the sub-window (each component is expressed as a fraction in the range [0,1]). */
  Vector2f m_bottomRight;

  /** The image in which to store the scene visualisation for the sub-window. */
  ITMUChar4Image_Ptr m_image;

  /** The location of the top-left of the sub-window (each component is expressed as a fraction in the range [0,1]). */
  Vector2f m_topLeft;

  /** The type of scene visualisation to render in the sub-window. */
  Raycaster::RaycastType m_type;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a sub-window.
   *
   * \param topLeft     The location of the top-left of the sub-window (each component is expressed as a fraction in the range [0,1]).
   * \param bottomRight The location of the bottom-right of the sub-window (each component is expressed as a fraction in the range [0,1]).
   * \param type        The type of scene visualisation to render in the sub-window.
   * \param imgSize     The size of image needed to store the scene visualisation for the sub-window.
   */
  Subwindow(const Vector2f& topLeft, const Vector2f& bottomRight, Raycaster::RaycastType type, const Vector2i& imgSize);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the location of the bottom-right of the sub-window (each component is expressed as a fraction in the range [0,1]).
   *
   * \return  The location of the bottom-right of the sub-window.
   */
  const Vector2f& bottom_right() const;

  /**
   * \brief Gets the image in which to store the scene visualisation for the sub-window.
   *
   * \return  The image in which to store the scene visualisation for the sub-window.
   */
  const ITMUChar4Image_Ptr& get_image();

  /**
   * \brief Gets the image in which to store the scene visualisation for the sub-window.
   *
   * \return  The image in which to store the scene visualisation for the sub-window.
   */
  ITMUChar4Image_CPtr get_image() const;

  /**
   * \brief Gets the type of scene visualisation to render in the sub-window.
   *
   * \return  The type of scene visualisation to render in the sub-window.
   */
  Raycaster::RaycastType get_type() const;

  /**
   * \brief Gets the height of the sub-window (as a fraction of the viewport height, in the range [0,1]).
   *
   * \return  The height of the sub-window.
   */
  float height() const;

  /**
   * \brief Sets the type of scene visualisation to render in the sub-window.
   *
   * \param type  The type of scene visualisation to render in the sub-window.
   */
  void set_type(Raycaster::RaycastType type);

  /**
   * \brief Gets the location of the top-left of the sub-window (each component is expressed as a fraction in the range [0,1]).
   *
   * \return  The location of the top-left of the sub-window.
   */
  const Vector2f& top_left() const;

  /**
   * \brief Gets the width of the sub-window (as a fraction of the viewport width, in the range [0,1]).
   *
   * \return  The width of the sub-window.
   */
  float width() const;
};

#endif
