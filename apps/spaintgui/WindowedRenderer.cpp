/**
 * spaintgui: WindowedRenderer.cpp
 */

#include "WindowedRenderer.h"

#include <stdexcept>

#include <rigging/SimpleCamera.h>
using namespace rigging;

#include <spaint/ogl/QuadricRenderer.h>
#include <spaint/selectiontransformers/interface/VoxelToCubeSelectionTransformer.h>
#ifdef WITH_LEAP
#include <spaint/selectors/LeapSelector.h>
#endif
#include <spaint/selectors/PickingSelector.h>
#include <spaint/util/CameraPoseConverter.h>
using namespace spaint;

//#################### LOCAL TYPES ####################

/**
 * \brief An instance of this class can be used to visit selectors in order to render them.
 */
class SelectorRenderer : public SelectionTransformerVisitor, public SelectorVisitor
{
  //~~~~~~~~~~~~~~~~~~~~ PRIVATE VARIABLES ~~~~~~~~~~~~~~~~~~~~
private:
  const WindowedRenderer *m_base;
  Vector3f m_colour;
  mutable int m_selectionRadius;

  //~~~~~~~~~~~~~~~~~~~~ CONSTRUCTORS ~~~~~~~~~~~~~~~~~~~~
public:
  SelectorRenderer(const WindowedRenderer *base, const Vector3f& colour)
  : m_base(base), m_colour(colour)
  {}

  //~~~~~~~~~~~~~~~~~~~~ PUBLIC MEMBER FUNCTIONS ~~~~~~~~~~~~~~~~~~~~
public:
#ifdef WITH_LEAP
  /** Override */
  virtual void visit(const LeapSelector& selector) const
  {
    const Leap::Frame& frame = selector.get_frame();
    if(!frame.isValid() || frame.hands().count() != 1) return;

    const Leap::Hand& hand = frame.hands()[0];
    for(int fingerIndex = 0, fingerCount = hand.fingers().count(); fingerIndex < fingerCount; ++fingerIndex)
    {
      const Leap::Finger& finger = hand.fingers()[fingerIndex];

      const int boneCount = 4;  // there are four bones per finger in the Leap hand model
      for(int boneIndex = 0; boneIndex < boneCount; ++boneIndex)
      {
        const Leap::Bone& bone = finger.bone(Leap::Bone::Type(boneIndex));

        glColor3f(0.8f, 0.8f, 0.8f);
        QuadricRenderer::render_cylinder(
          LeapSelector::from_leap_vector(bone.prevJoint()),
          LeapSelector::from_leap_vector(bone.nextJoint()),
          LeapSelector::from_leap_size(bone.width() * 0.5f),
          LeapSelector::from_leap_size(bone.width() * 0.5f),
          10
        );

        glColor3f(1.0f, 0.0f, 0.0f);
        QuadricRenderer::render_sphere(LeapSelector::from_leap_vector(bone.nextJoint()), LeapSelector::from_leap_size(bone.width() * 0.7f), 10, 10);
      }
    }
  }
#endif

  /** Override */
  virtual void visit(const PickingSelector& selector) const
  {
    boost::optional<Eigen::Vector3f> pickPoint = selector.get_position();
    if(!pickPoint) return;

    glColor3f(m_colour.r, m_colour.g, m_colour.b);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    QuadricRenderer::render_sphere(*pickPoint, m_selectionRadius * m_base->m_model->get_settings()->sceneParams.voxelSize, 10, 10);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  }

  /** Override */
  virtual void visit(const VoxelToCubeSelectionTransformer& transformer) const
  {
    m_selectionRadius = transformer.get_radius();
  }
};

//#################### CONSTRUCTORS ####################

WindowedRenderer::WindowedRenderer(const spaint::SpaintModel_CPtr& model, const spaint::SpaintRaycaster_CPtr& raycaster,
                                   const std::string& title, int width, int height)
: Renderer(model, raycaster), m_height(height), m_width(width)
{
  // Create the window into which to render.
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

  m_window.reset(
    SDL_CreateWindow(
      title.c_str(),
      SDL_WINDOWPOS_UNDEFINED,
      SDL_WINDOWPOS_UNDEFINED,
      width,
      height,
      SDL_WINDOW_OPENGL
    ),
    &SDL_DestroyWindow
  );

  m_context.reset(
    SDL_GL_CreateContext(m_window.get()),
    SDL_GL_DeleteContext
  );

  glViewport(0, 0, width, height);

  glDepthFunc(GL_LEQUAL);
  glEnable(GL_DEPTH_TEST);

  // Set up the camera.
  m_camera.reset(new SimpleCamera(Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 0.0f, 1.0f), Eigen::Vector3f(0.0f, -1.0f, 0.0f)));

  // Set up a texture in which to store the reconstructed scene.
  glGenTextures(1, &m_textureID);
}

//#################### DESTRUCTOR ####################

WindowedRenderer::~WindowedRenderer()
{
  glDeleteTextures(1, &m_textureID);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

rigging::MoveableCamera_Ptr WindowedRenderer::get_camera()
{
  return m_camera;
}

WindowedRenderer::RenderState_CPtr WindowedRenderer::get_monocular_render_state() const
{
  return m_renderState;
}

void WindowedRenderer::render(const SpaintInteractor_CPtr& interactor) const
{
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Determine the camera pose.
  ITMPose pose;
  switch(m_cameraMode)
  {
    case CM_FOLLOW:
      pose = m_model->get_pose();
      break;
    case CM_FREE:
      pose = CameraPoseConverter::camera_to_pose(*m_camera);
      break;
    default:
      // This should never happen.
      throw std::runtime_error("Error: Unknown camera mode");
  }

  // Render the reconstructed scene, then render a synthetic scene over the top of it.
  render_reconstructed_scene(pose);
  render_synthetic_scene(pose, interactor);

  SDL_GL_SwapWindow(m_window.get());
}

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

void WindowedRenderer::render_reconstructed_scene(const ITMPose& pose) const
{
  // Raycast the scene.
  m_raycaster->generate_free_raycast(m_image, m_renderState, pose, m_phongEnabled ? SpaintRaycaster::RT_SEMANTICPHONG : SpaintRaycaster::RT_SEMANTICLAMBERTIAN);

  // Draw a quad textured with the raycasted scene.
  begin_2d();
    glEnable(GL_TEXTURE_2D);
    {
      glBindTexture(GL_TEXTURE_2D, m_textureID);
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m_image->noDims.x, m_image->noDims.y, 0, GL_RGBA, GL_UNSIGNED_BYTE, m_image->GetData(MEMORYDEVICE_CPU));
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glColor3f(1.0f, 1.0f, 1.0f);
      glBegin(GL_QUADS);
      {
        glTexCoord2f(0, 1); glVertex2f(0, 0);
        glTexCoord2f(1, 1); glVertex2f(1, 0);
        glTexCoord2f(1, 0); glVertex2f(1, 1);
        glTexCoord2f(0, 0); glVertex2f(0, 1);
      }
      glEnd();
    }
    glDisable(GL_TEXTURE_2D);
  end_2d();
}

void WindowedRenderer::render_synthetic_scene(const ITMPose& pose, const SpaintInteractor_CPtr& interactor) const
{
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  {
    set_projection_matrix(m_model->get_intrinsics(), m_width, m_height);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    {
      // Note: Conveniently, data() returns the elements in column-major order (the order required by OpenGL).
      glLoadMatrixf(CameraPoseConverter::pose_to_modelview(pose).data());

      // Render the axes.
      glBegin(GL_LINES);
        glColor3f(1.0f, 0.0f, 0.0f);  glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(1.0f, 0.0f, 0.0f);
        glColor3f(0.0f, 1.0f, 0.0f);  glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(0.0f, 1.0f, 0.0f);
        glColor3f(0.0f, 0.0f, 1.0f);  glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(0.0f, 0.0f, 1.0f);
      glEnd();

      // Render the current selector to show how we're interacting with the scene.
      Vector3u labelColour = m_model->get_label_manager()->get_label_colour(interactor->get_semantic_label());
      Vector3f selectorColour(labelColour.r / 255.0f, labelColour.g / 255.0f, labelColour.b / 255.0f);
      SelectorRenderer selectorRenderer(this, selectorColour);
      SpaintInteractor::SelectionTransformer_CPtr transformer = interactor->get_selection_transformer();
      if(transformer) transformer->accept(selectorRenderer);
      interactor->get_selector()->accept(selectorRenderer);
    }
    glPopMatrix();
  }
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
}
