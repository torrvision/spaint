/**
 * spaintgui: Renderer.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "Renderer.h"
using namespace ITMLib;
using namespace ORUtils;

#include <spaint/ogl/QuadricRenderer.h>
#include <spaint/selectiontransformers/interface/VoxelToCubeSelectionTransformer.h>
#include <spaint/selectors/PickingSelector.h>
#include <spaint/util/CameraPoseConverter.h>

#ifdef WITH_ARRAYFIRE
#include <spaint/imageprocessing/MedianFilterer.h>
#include <spaint/selectors/TouchSelector.h>
#endif

#ifdef WITH_LEAP
#include <spaint/selectors/LeapSelector.h>
#endif

using namespace spaint;

//#################### LOCAL TYPES ####################

/**
 * \brief An instance of this class can be used to visit selectors in order to render them.
 */
class SelectorRenderer : public SelectionTransformerVisitor, public SelectorVisitor
{
  //~~~~~~~~~~~~~~~~~~~~ TYPEDEFS ~~~~~~~~~~~~~~~~~~~~
private:
  typedef boost::shared_ptr<const ITMUChar4Image> ITMUChar4Image_CPtr;

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

    render_orb(*pickPoint, m_selectionRadius * m_base->m_model->get_settings()->sceneParams.voxelSize);
  }

#ifdef WITH_ARRAYFIRE
  /** Override */
  virtual void visit(const TouchSelector& selector) const
  {
    // Render a colour image containing the current touch interaction.
    render_touch_image(selector.generate_touch_image(m_base->m_model->get_view()));

    // Render the points at which the user is touching the scene.
    const int selectionRadius = 1;
    std::vector<Eigen::Vector3f> touchPoints = selector.get_positions();

    for(size_t i = 0, size = touchPoints.size(); i < size; ++i)
    {
      render_orb(touchPoints[i], selectionRadius * m_base->m_model->get_settings()->sceneParams.voxelSize);
    }

    // Render a rotating, coloured orb at the top-right of the viewport to indicate the current semantic label.
    const Vector2i& depthImageSize = m_base->m_model->get_depth_image_size();
    const float aspectRatio = static_cast<float>(depthImageSize.x) / depthImageSize.y;

    const Eigen::Vector3f labelOrbPos(0.9f, aspectRatio * 0.1f, 0.0f);
    const double labelOrbRadius = 0.05;

    static float angle = 0.0f;
    angle = fmod(angle + 5.0f, 360.0f);

    m_base->begin_2d();
      glTranslatef(labelOrbPos.x(), labelOrbPos.y(), labelOrbPos.z());
      glScalef(1.0f, aspectRatio, 1.0f);
      glRotatef(angle, 1.0f, 1.0f, 0.0f);
      glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
      glTranslatef(-labelOrbPos.x(), -labelOrbPos.y(), -labelOrbPos.z());

      glPushAttrib(GL_LINE_WIDTH);
      glLineWidth(2.0f);
        render_orb(labelOrbPos, labelOrbRadius);
      glPopAttrib();
    m_base->end_2d();
  }
#endif

  /** Override */
  virtual void visit(const VoxelToCubeSelectionTransformer& transformer) const
  {
    m_selectionRadius = transformer.get_radius();
  }

  //~~~~~~~~~~~~~~~~~~~~ PRIVATE MEMBER FUNCTIONS ~~~~~~~~~~~~~~~~~~~~
private:
  /**
   * \brief Renders an orb with a colour denoting the current semantic label.
   *
   * \param centre  The position of the centre of the orb.
   * \param radius  The radius of the orb.
   */
  void render_orb(const Eigen::Vector3f& centre, double radius) const
  {
    glColor3f(m_colour.r, m_colour.g, m_colour.b);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    QuadricRenderer::render_sphere(centre, radius, 10, 10);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  }
  
  /**
   * \brief Renders a colour image containing the current touch interaction.
   *
   * The rendering works by drawing a semi-transparent quad textured with the touch image over the existing scene.
   *
   * \param touchImage  A colour image containing the current touch interaction.
   */
  void render_touch_image(const ITMUChar4Image_CPtr& touchImage) const
  {
    // Copy the touch image to a texture.
    glBindTexture(GL_TEXTURE_2D, m_base->m_textureID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, touchImage->noDims.x, touchImage->noDims.y, 0, GL_RGBA, GL_UNSIGNED_BYTE, touchImage->GetData(MEMORYDEVICE_CPU));
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    // Enable blending.
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Render a semi-transparent quad textured with the touch image over the top of the existing scene.
    m_base->begin_2d();
      m_base->render_textured_quad(m_base->m_textureID);
    m_base->end_2d();

    // Disable blending again.
    glDisable(GL_BLEND);
  }
};

//#################### CONSTRUCTORS ####################

Renderer::Renderer(const Model_CPtr& model, const Raycaster_CPtr& raycaster)
: m_cameraMode(CM_FOLLOW),
  m_medianFilteringEnabled(true),
  m_model(model),
  m_raycaster(raycaster)
{}

//#################### DESTRUCTOR ####################

Renderer::~Renderer() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

boost::optional<Vector2f> Renderer::compute_fractional_position(int x, int y) const
{
  boost::optional<size_t> subwindowIndex = determine_subwindow_index(x, y);
  if(!subwindowIndex) return boost::none;

  // FIXME: This is the same as in determine_subwindow_index. It should be factored out.
  const Vector2f viewportSize = m_model->get_depth_image_size().toFloat();
  const float viewportFracX = x / (viewportSize.x - 1), viewportFracY = y / (viewportSize.y - 1);

  const Subwindow& subwindow = (*m_subwindowConfiguration)[*subwindowIndex];
  const Vector2f& tl = subwindow.m_topLeft;
  const Vector2f& br = subwindow.m_bottomRight;

  return Vector2f(
    CLAMP((viewportFracX - tl.x) / (br.x - tl.x), 0.0f, 1.0f),
    CLAMP((viewportFracY - tl.y) / (br.y - tl.y), 0.0f, 1.0f)
  );
}

boost::optional<size_t> Renderer::determine_subwindow_index(int x, int y) const
{
  Vector2f viewportSize = m_model->get_depth_image_size().toFloat();
  float viewportFracX = x / (viewportSize.x - 1), viewportFracY = y / (viewportSize.y - 1);

  for(size_t i = 0, count = m_subwindowConfiguration->size(); i < count; ++i)
  {
    const Subwindow& subwindow = (*m_subwindowConfiguration)[i];
    const Vector2f& tl = subwindow.m_topLeft;
    const Vector2f& br = subwindow.m_bottomRight;
    if(tl.x <= viewportFracX && viewportFracX <= br.x &&
       tl.y <= viewportFracY && viewportFracY <= br.y)
    {
      return i;
    }
  }

  return boost::none;
}

Renderer::CameraMode Renderer::get_camera_mode() const
{
  return m_cameraMode;
}

bool Renderer::get_median_filtering_enabled() const
{
  return m_medianFilteringEnabled;
}

size_t Renderer::get_subwindow_count() const
{
  return m_subwindowConfiguration->size();
}

void Renderer::reset_subwindow_configuration()
{
  m_subwindowConfiguration = make_default_subwindow_configuration(m_subwindowConfiguration->size());
}

void Renderer::set_camera_mode(CameraMode cameraMode)
{
  m_cameraMode = cameraMode;
}

void Renderer::set_median_filtering_enabled(bool medianFilteringEnabled)
{
  m_medianFilteringEnabled = medianFilteringEnabled;
}

void Renderer::set_subwindow_configuration(size_t i)
{
  // If the saved configuration index is valid:
  if(i < m_savedSubwindowConfigurations.size())
  {
    // If the saved configuration is null, try to create a default one of the right size.
    if(!m_savedSubwindowConfigurations[i]) m_savedSubwindowConfigurations[i] = make_default_subwindow_configuration(i);

    // If the saved configuration was already or is now valid, use it.
    if(m_savedSubwindowConfigurations[i]) m_subwindowConfiguration = m_savedSubwindowConfigurations[i];
  }
}

void Renderer::set_subwindow_type(size_t subwindowIndex, Raycaster::RaycastType type)
{
  // Note: A null sub-window configuration should never be active.
  assert(m_subwindowConfiguration);

  if(subwindowIndex < m_subwindowConfiguration->size())
  {
    (*m_subwindowConfiguration)[subwindowIndex].m_type = type;
  }
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
  m_subwindowConfiguration.reset();
  std::vector<SubwindowConfiguration_Ptr>().swap(m_savedSubwindowConfigurations);
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

Model_CPtr Renderer::get_model() const
{
  return m_model;
}

SDL_Window *Renderer::get_window() const
{
  return m_window.get();
}

void Renderer::initialise_common()
{
  // Set up a texture in which to temporarily store scene visualisations and the touch image when rendering.
  glGenTextures(1, &m_textureID);

  // Set up the sub-window configurations.
  for(int i = 0; i <= 3; ++i)
  {
    m_savedSubwindowConfigurations.push_back(make_default_subwindow_configuration(i));
  }

  // Set the initial sub-window configuration.
  set_subwindow_configuration(3);
}

void Renderer::render_scene(const SE3Pose& pose, const Interactor_CPtr& interactor, Raycaster::RenderState_Ptr& renderState) const
{
  // Set the viewport for the window.
  ORUtils::Vector2<int> depthImageSize = m_model->get_depth_image_size();
  glViewport(0, 0, depthImageSize.width, depthImageSize.height);

  // Clear the frame buffer.
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // If we have started reconstruction, render all the sub-windows.
  if(m_model->get_view())
  {
    for(size_t subwindowIndex = 0, count = m_subwindowConfiguration->size(); subwindowIndex < count; ++subwindowIndex)
    {
      // Set the viewport for the sub-window.
      const Subwindow& subwindow = (*m_subwindowConfiguration)[subwindowIndex];
      int x = (int)ROUND(subwindow.m_topLeft.x * depthImageSize.width);
      int y = (int)ROUND((1 - subwindow.m_bottomRight.y) * depthImageSize.height);
      int width = (int)ROUND((subwindow.m_bottomRight.x - subwindow.m_topLeft.x) * depthImageSize.width);
      int height = (int)ROUND((subwindow.m_bottomRight.y - subwindow.m_topLeft.y) * depthImageSize.height);
      glViewport(x, y, width, height);

      // Render the reconstructed scene, then render a synthetic scene over the top of it.
      render_reconstructed_scene(pose, renderState, subwindowIndex);
      render_synthetic_scene(pose, interactor, subwindowIndex);
    }
  }
}

void Renderer::set_window(const SDL_Window_Ptr& window)
{
  m_window = window;

  // Create an OpenGL context for the window.
  m_context.reset(
    SDL_GL_CreateContext(m_window.get()),
    SDL_GL_DeleteContext
  );

  // Initialise GLEW (if necessary).
#ifdef WITH_GLEW
  GLenum err = glewInit();
  if(err != GLEW_OK) throw std::runtime_error("Error: Could not initialise GLEW");
#endif
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

Renderer::SubwindowConfiguration_Ptr Renderer::make_default_subwindow_configuration(size_t subwindowCount) const
{
  SubwindowConfiguration_Ptr config;

  switch(subwindowCount)
  {
    case 1:
    {
      config.reset(new SubwindowConfiguration(1));
      (*config)[0].m_topLeft = Vector2f(0, 0);
      (*config)[0].m_bottomRight = Vector2f(1, 1);
      (*config)[0].m_type = Raycaster::RT_SEMANTICLAMBERTIAN;
      break;
    }
    case 3:
    {
      const float x = 0.665f;
      const float y = 0.5f;
      config.reset(new SubwindowConfiguration(3));
      (*config)[0].m_topLeft = Vector2f(0, 0);
      (*config)[0].m_bottomRight = Vector2f(x, y * 2);
      (*config)[0].m_type = Raycaster::RT_SEMANTICLAMBERTIAN;
      (*config)[1].m_topLeft = Vector2f(x, 0);
      (*config)[1].m_bottomRight = Vector2f(1, y);
      (*config)[1].m_type = Raycaster::RT_SEMANTICCOLOUR;
      (*config)[2].m_topLeft = Vector2f(x, y);
      (*config)[2].m_bottomRight = Vector2f(1, y * 2);
      (*config)[2].m_type = Raycaster::RT_SEMANTICPHONG;
      break;
    }
    default:
      break;
  }

  if(config)
  {
    for(size_t i = 0; i < config->size(); ++i)
    {
      (*config)[i].m_image.reset(new ITMUChar4Image(m_model->get_depth_image_size(), true, true));
    }
  }

  return config;
}

void Renderer::render_reconstructed_scene(const SE3Pose& pose, Raycaster::RenderState_Ptr& renderState, size_t subwindowIndex) const
{
  // Set up any post-processing that needs to be applied to the raycast result.
  // FIXME: At present, median filtering breaks in CPU mode, so we prevent it from running, but we should investigate why.
  static boost::optional<Raycaster::Postprocessor> postprocessor = boost::none;
  if(!m_medianFilteringEnabled && postprocessor)
  {
    postprocessor.reset();
  }
  else if(m_medianFilteringEnabled && !postprocessor && m_model->get_settings()->deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#if defined(WITH_ARRAYFIRE) && !defined(USE_LOW_POWER_MODE)
    const unsigned int kernelWidth = 3;
    postprocessor = MedianFilterer(kernelWidth, m_model->get_settings()->deviceType);
#endif
  }

  begin_2d();

  // Generate the subwindow image.
  const Subwindow& subwindow = (*m_subwindowConfiguration)[subwindowIndex];
  m_raycaster->generate_free_raycast(subwindow.m_image, renderState, pose, subwindow.m_type, postprocessor);

  // Copy the raycasted scene to a texture.
  glBindTexture(GL_TEXTURE_2D, m_textureID);
  glTexImage2D(
    GL_TEXTURE_2D, 0, GL_RGBA, subwindow.m_image->noDims.x, subwindow.m_image->noDims.y, 0, GL_RGBA, GL_UNSIGNED_BYTE,
    subwindow.m_image->GetData(MEMORYDEVICE_CPU)
  );
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

  // Render a quad textured with the subwindow image.
  render_textured_quad(m_textureID);

  end_2d();
}

void Renderer::render_synthetic_scene(const SE3Pose& pose, const Interactor_CPtr& interactor, size_t subwindowIndex) const
{
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_DEPTH_TEST);

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
      Interactor::SelectionTransformer_CPtr transformer = interactor->get_selection_transformer();
      if(transformer) transformer->accept(selectorRenderer);
      interactor->get_selector()->accept(selectorRenderer);
    }
    glPopMatrix();
  }
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();

  glDisable(GL_DEPTH_TEST);
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
