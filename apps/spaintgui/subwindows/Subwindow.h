/**
 * spaintgui: Subwindow.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINTGUI_SUBWINDOW
#define H_SPAINTGUI_SUBWINDOW

#include <rigging/CompositeCamera.h>

#include <spaint/visualisation/VisualisationGenerator.h>

/**
  * \brief An instance of this class can be used to represent a sub-window into
  *        which different types of scene visualisation can be rendered.
  */
class Subwindow
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<ITMLib::ITMRenderState> RenderState_Ptr;
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;

  //#################### ENUMERATIONS ####################
public:
  /**
   * \brief An enumeration containing the possible camera modes we can use.
   */
  enum CameraMode
  {
    /** A mode that follows the camera that is reconstructing the scene. */
    CM_FOLLOW,

    /** A mode that allows the user to freely move the camera around to view the scene from different angles. */
    CM_FREE
  };

  //#################### PRIVATE VARIABLES ####################
private:
  /** The location of the bottom-right of the sub-window (each component is expressed as a fraction in the range [0,1]). */
  Vector2f m_bottomRight;

  /** The camera from which to render the scene. */
  rigging::CompositeCamera_Ptr m_camera;

  /** The current camera mode. */
  CameraMode m_cameraMode;

  /** The image in which to store the scene visualisation for the sub-window. */
  ITMUChar4Image_Ptr m_image;

  /** The render state(s) for the free camera view(s). */
  mutable std::map<int,RenderState_Ptr> m_renderStates;

  /** The ID of the scene to render in the sub-window. */
  std::string m_sceneID;

  /** The location of the top-left of the sub-window (each component is expressed as a fraction in the range [0,1]). */
  Vector2f m_topLeft;

  /** The type of scene visualisation to render in the sub-window. */
  spaint::VisualisationGenerator::VisualisationType m_type;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a sub-window.
   *
   * \param topLeft     The location of the top-left of the sub-window (each component is expressed as a fraction in the range [0,1]).
   * \param bottomRight The location of the bottom-right of the sub-window (each component is expressed as a fraction in the range [0,1]).
   * \param sceneID     The ID of the scene to render in the sub-window.
   * \param type        The type of scene visualisation to render in the sub-window.
   * \param imgSize     The size of image needed to store the scene visualisation for the sub-window.
   */
  Subwindow(const Vector2f& topLeft, const Vector2f& bottomRight, const std::string& sceneID, spaint::VisualisationGenerator::VisualisationType type, const Vector2i& imgSize);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the location of the bottom-right of the sub-window (each component is expressed as a fraction in the range [0,1]).
   *
   * \return  The location of the bottom-right of the sub-window.
   */
  const Vector2f& bottom_right() const;

  /**
   * \brief Gets the camera from which to render the scene.
   *
   * \return  The camera from which to render the scene.
   */
  const rigging::CompositeCamera_Ptr& get_camera() const;

  /**
   * \brief Gets the current camera mode.
   *
   * \return  The current camera mode.
   */
  CameraMode get_camera_mode() const;

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
   * \brief Gets the render state for the specified free camera view for the sub-window.
   *
   * \param viewIndex The index of the free camera view for the sub-window.
   */
  RenderState_Ptr& get_render_state(int viewIndex);

  /**
   * \brief Gets the render state for the specified free camera view for the sub-window.
   *
   * \param viewIndex The index of the free camera view for the sub-window.
   */
  RenderState_CPtr get_render_state(int viewIndex) const;

  /**
   * \brief Gets the ID of the scene to render in the sub-window.
   *
   * \return  The ID of the scene to render in the sub-window.
   */
  const std::string& get_scene_id() const;

  /**
   * \brief Gets the type of scene visualisation to render in the sub-window.
   *
   * \return  The type of scene visualisation to render in the sub-window.
   */
  spaint::VisualisationGenerator::VisualisationType get_type() const;

  /**
   * \brief Gets the height of the sub-window (as a fraction of the window viewport height, in the range [0,1]).
   *
   * \return  The height of the sub-window.
   */
  float height() const;

  /**
   * \brief Resets the camera from which to render the scene.
   *
   * \param camera  The camera from which to render the scene.
   */
  void reset_camera();

  /**
   * \brief Sets the current camera mode.
   *
   * \param cameraMode  The new camera mode.
   */
  void set_camera_mode(CameraMode cameraMode);

  /**
   * \brief Sets the type of scene visualisation to render in the sub-window.
   *
   * \param type  The type of scene visualisation to render in the sub-window.
   */
  void set_type(spaint::VisualisationGenerator::VisualisationType type);

  /**
   * \brief Gets the location of the top-left of the sub-window (each component is expressed as a fraction in the range [0,1]).
   *
   * \return  The location of the top-left of the sub-window.
   */
  const Vector2f& top_left() const;

  /**
   * \brief Gets the width of the sub-window (as a fraction of the window viewport width, in the range [0,1]).
   *
   * \return  The width of the sub-window.
   */
  float width() const;
};

#endif
