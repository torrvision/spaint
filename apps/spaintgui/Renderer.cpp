/**
 * spaintgui: Renderer.cpp
 */

#include "Renderer.h"

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
  const Renderer *m_base;
  Vector3f m_colour;
  mutable int m_selectionRadius;

  //~~~~~~~~~~~~~~~~~~~~ CONSTRUCTORS ~~~~~~~~~~~~~~~~~~~~
public:
  SelectorRenderer(const Renderer *base, const Vector3f& colour)
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

Renderer::Renderer(const spaint::SpaintModel_CPtr& model, const spaint::SpaintRaycaster_CPtr& raycaster)
: m_cameraMode(CM_FOLLOW), m_model(model), m_phongEnabled(false), m_raycaster(raycaster)
{}

//#################### DESTRUCTOR ####################

Renderer::~Renderer() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

Renderer::CameraMode Renderer::get_camera_mode() const
{
  return m_cameraMode;
}

bool Renderer::get_phong_enabled() const
{
  return m_phongEnabled;
}

void Renderer::set_camera_mode(CameraMode cameraMode)
{
  m_cameraMode = cameraMode;
}

void Renderer::set_phong_enabled(bool phongEnabled)
{
  m_phongEnabled = phongEnabled;
}

//#################### PROTECTED MEMBER FUNCTIONS ####################

void Renderer::begin_2d()
{
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(0.0, 1.0, 0.0, 1.0, 0.0, 1.0);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glTranslated(0.0, 1.0, 0.0);
  glScaled(1.0, -1.0, 1.0);

  glDepthMask(false);
}

void Renderer::destroy_common()
{
  glDeleteTextures(1, &m_textureID);
}

void Renderer::end_2d()
{
  glDepthMask(true);

  // We assume that the matrix mode is still set to GL_MODELVIEW at the start of this function.
  glPopMatrix();

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
}

void Renderer::initialise_common()
{
  // Create an image into which to temporarily store visualisations of the scene.
  m_image.reset(new ITMUChar4Image(m_model->get_depth_image_size(), true, true));

  // Set up a texture in which to store the reconstructed scene.
  glGenTextures(1, &m_textureID);
}

void Renderer::render_scene(const ITMPose& pose, const spaint::SpaintInteractor_CPtr& interactor, spaint::SpaintRaycaster::RenderState_Ptr& renderState) const
{
  // Clear the frame buffer.
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Render the reconstructed scene, then render a synthetic scene over the top of it.
  render_reconstructed_scene(pose, renderState);
  render_synthetic_scene(pose, interactor);
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void Renderer::render_reconstructed_scene(const ITMPose& pose, spaint::SpaintRaycaster::RenderState_Ptr& renderState) const
{
  // Raycast the scene.
  m_raycaster->generate_free_raycast(m_image, renderState, pose, m_phongEnabled ? SpaintRaycaster::RT_SEMANTICPHONG : SpaintRaycaster::RT_SEMANTICLAMBERTIAN);

  // Copy the raycasted scene to a texture.
  glBindTexture(GL_TEXTURE_2D, m_textureID);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m_image->noDims.x, m_image->noDims.y, 0, GL_RGBA, GL_UNSIGNED_BYTE, m_image->GetData(MEMORYDEVICE_CPU));
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

  // Render a quad textured with the raycasted scene.
  begin_2d();
    render_textured_quad(m_textureID);
  end_2d();
}

void Renderer::render_synthetic_scene(const ITMPose& pose, const SpaintInteractor_CPtr& interactor) const
{
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  {
    ORUtils::Vector2<int> depthImageSize = m_model->get_depth_image_size();
    set_projection_matrix(m_model->get_intrinsics(), depthImageSize.width, depthImageSize.height);

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

void Renderer::render_textured_quad(GLuint textureID)
{
  glEnable(GL_TEXTURE_2D);
  {
    glBindTexture(GL_TEXTURE_2D, textureID);
    glColor3f(1.0f, 1.0f, 1.0f);
    glBegin(GL_QUADS);
    {
      glTexCoord2f(0, 0); glVertex2f(0, 0);
      glTexCoord2f(1, 0); glVertex2f(1, 0);
      glTexCoord2f(1, 1); glVertex2f(1, 1);
      glTexCoord2f(0, 1); glVertex2f(0, 1);
    }
    glEnd();
  }
  glDisable(GL_TEXTURE_2D);
}

void Renderer::set_projection_matrix(const ITMIntrinsics& intrinsics, int width, int height)
{
  double nearVal = 0.1;
  double farVal = 1000.0;

  // To rederive these equations, use similar triangles. Note that fx = f / sx and fy = f / sy,
  // where sx and sy are the dimensions of a pixel on the image plane.
  double leftVal = -intrinsics.projectionParamsSimple.px * nearVal / intrinsics.projectionParamsSimple.fx;
  double rightVal = (width - intrinsics.projectionParamsSimple.px) * nearVal / intrinsics.projectionParamsSimple.fx;
  double bottomVal = -intrinsics.projectionParamsSimple.py * nearVal / intrinsics.projectionParamsSimple.fy;
  double topVal = (height - intrinsics.projectionParamsSimple.py) * nearVal / intrinsics.projectionParamsSimple.fy;

  glLoadIdentity();
  glFrustum(leftVal, rightVal, bottomVal, topVal, nearVal, farVal);
}
