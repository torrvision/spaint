/**
 * spaintgui: RiftRenderer.h
 */

#ifndef H_SPAINTGUI_RIFTRENDERER
#define H_SPAINTGUI_RIFTRENDERER

#include <string>

#include <OVR.h>

#include <spaint/ogl/WrappedGL.h>

#include "Renderer.h"

/**
 * \brief An instance of this class can be used to render the scene constructed by the spaint engine to the Oculus Rift.
 */
class RiftRenderer : public Renderer
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The eye texture IDs. */
  GLuint m_eyeTextureIDs[ovrEye_Count];

  /** The Rift handle. */
  ovrHmd m_hmd;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a Rift renderer.
   *
   * \param title     The title to give the window.
   * \param windowed  Whether or not to render in a window rather than on the Rift itself.
   */
  RiftRenderer(const std::string& title, bool windowed);

  //#################### DESTRUCTOR ####################
public:
  ~RiftRenderer();

  //#################### COPY CONSTRUCTOR & ASSIGNMENT OPERATOR ####################
private:
  // Deliberately private and unimplemented.
  RiftRenderer(const RiftRenderer&);
  RiftRenderer& operator=(const RiftRenderer&);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void render(const spaint::SpaintEngine_Ptr& spaintEngine) const;
};

#endif
