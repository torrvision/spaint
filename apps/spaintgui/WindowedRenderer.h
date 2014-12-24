/**
 * spaintgui: WindowedRenderer.h
 */

#ifndef H_SPAINTGUI_WINDOWEDRENDERER
#define H_SPAINTGUI_WINDOWEDRENDERER

#include <string>

#include <spaint/ogl/WrappedGL.h>

#include "Renderer.h"

/**
 * \brief An instance of this class can be used to render the scene constructed by the spaint engine to a window.
 */
class WindowedRenderer : public Renderer
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The height of the window. */
  int m_height;

  /** The image in which to store the visualisation each frame. */
  ITMUChar4Image_Ptr m_image;

  /** The texture ID for the visualisation we're drawing. */
  GLuint m_textureID;

  /** The width of the window. */
  int m_width;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a windowed renderer.
   *
   * \param spaintEngine  The spaint engine.
   * \param title         The title of the window.
   * \param width         The width of the window.
   * \param height        The height of the window.
   */
  WindowedRenderer(const spaint::SpaintEngine_Ptr& spaintEngine, const std::string& title, int width, int height);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the renderer.
   */
  ~WindowedRenderer();

  //#################### COPY CONSTRUCTOR & ASSIGNMENT OPERATOR ####################
private:
  // Deliberately private and unimplemented.
  WindowedRenderer(const WindowedRenderer&);
  WindowedRenderer& operator=(const WindowedRenderer&);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void render(const spaint::Camera_CPtr& camera) const;

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Calculates the InfiniTAM pose of the specified camera.
   *
   * \param camera  The camera.
   * \return        The InfiniTAM pose of the camera.
   */
  static ITMPose calculate_pose(const spaint::Camera& camera);

  /**
   * \brief Sets the OpenGL model-view matrix corresponding to the specified InfiniTAM pose.
   *
   * \param pose  The InfiniTAM pose.
   */
  static void set_modelview_matrix(const ITMPose& pose);

  /**
   * \brief Sets the OpenGL projection matrix based on a set of intrinsic camera parameters.
   *
   * \param intrinsics  The intrinsic camera parameters.
   * \param width       The width of the viewport.
   * \param height      The height of the viewport.
   */
  static void set_projection_matrix(const ITMIntrinsics& intrinsics, int width, int height);
};

#endif
