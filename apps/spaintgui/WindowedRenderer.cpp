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
#ifdef WITH_ARRAYFIRE
#include <spaint/selectors/TouchSelector.h>
#endif
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

  /** Override */
  virtual void visit(const TouchSelector& selector) const
  {
    // FIXME This is an exact copy of the picking selector. What should be rendered at the touch points?
    boost::optional<Eigen::Vector3f> pickPoint = selector.get_position();
    if(!pickPoint) return;

    glColor3f(1.0f, 0.0f, 1.0f);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    QuadricRenderer::render_sphere(*pickPoint, m_selectionRadius * m_base->m_model->get_settings()->sceneParams.voxelSize, 10, 10);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  }
};

//#################### CONSTRUCTORS ####################

WindowedRenderer::WindowedRenderer(const std::string& title, const spaint::SpaintModel_CPtr& model, const spaint::SpaintRaycaster_CPtr& raycaster)
: Renderer(model, raycaster)
{
  // Create the window into which to render.
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

  ORUtils::Vector2<int> depthImageSize = get_model()->get_depth_image_size();
  int width = depthImageSize.width, height = depthImageSize.height;

  set_window(SDL_Window_Ptr(
    SDL_CreateWindow(
      title.c_str(),
      SDL_WINDOWPOS_UNDEFINED,
      SDL_WINDOWPOS_UNDEFINED,
      width,
      height,
      SDL_WINDOW_OPENGL
    ),
    &SDL_DestroyWindow
  ));

  // Set up the camera.
  m_camera.reset(new SimpleCamera(Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 0.0f, 1.0f), Eigen::Vector3f(0.0f, -1.0f, 0.0f)));

  // Initialise the temporary image and texture used for visualising the scene.
  initialise_common();
}

//#################### DESTRUCTOR ####################

WindowedRenderer::~WindowedRenderer()
{
  destroy_common();
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
  // Determine the camera pose.
  ITMPose pose;
  switch(get_camera_mode())
  {
    case CM_FOLLOW:
      pose = get_model()->get_pose();
      break;
    case CM_FREE:
      pose = CameraPoseConverter::camera_to_pose(*m_camera);
      break;
    default:
      // This should never happen.
      throw std::runtime_error("Error: Unknown camera mode");
  }

  // Render the scene from that camera pose.
  render_scene(pose, interactor, m_renderState);

  SDL_GL_SwapWindow(get_window());
}
