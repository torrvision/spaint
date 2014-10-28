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
  /** A pointer to an array of eye texture IDs. */
  spaint::shared_ptr<GLuint> m_eyeTextureIDs;

  /** The Rift handle. */
  ovrHmd m_hmd;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a Rift renderer.
   *
   * \param title The title to give the window.
   */
  explicit RiftRenderer(const std::string& title);

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
