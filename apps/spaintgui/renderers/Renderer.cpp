/**
 * spaintgui: Renderer.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "Renderer.h"
using namespace ITMLib;
using namespace ORUtils;
using namespace rigging;

#include <spaint/ogl/CameraRenderer.h>
#include <spaint/ogl/QuadricRenderer.h>
#include <spaint/selectiontransformers/interface/VoxelToCubeSelectionTransformer.h>
#include <spaint/selectors/PickingSelector.h>
#include <spaint/util/CameraFactory.h>
#include <spaint/util/CameraPoseConverter.h>
using namespace spaint;

#ifdef WITH_ARRAYFIRE
#include <spaint/imageprocessing/MedianFilterer.h>
#include <spaint/selectors/TouchSelector.h>
#endif

#ifdef WITH_LEAP
#include <spaint/selectors/LeapSelector.h>
#endif

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
    // Render the current touch interaction as an overlay.
    m_base->render_overlay(selector.generate_touch_image(m_base->m_model->get_slam_state(Model::get_world_scene_id())->get_view()));

    // Render the points at which the user is touching the scene.
    const int selectionRadius = 1;
    std::vector<Eigen::Vector3f> touchPoints = selector.get_positions();

    for(size_t i = 0, size = touchPoints.size(); i < size; ++i)
    {
      render_orb(touchPoints[i], selectionRadius * m_base->m_model->get_settings()->sceneParams.voxelSize);
    }

    // Render a rotating, coloured orb at the top-right of the viewport to indicate the current semantic label.
    const Vector2i& depthImageSize = m_base->m_model->get_slam_state(Model::get_world_scene_id())->get_depth_image_size();
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
};

//#################### CONSTRUCTORS ####################

Renderer::Renderer(const Model_CPtr& model, const SubwindowConfiguration_Ptr& subwindowConfiguration, const Vector2i& windowViewportSize)
: m_medianFilteringEnabled(true),
  m_model(model),
  m_subwindowConfiguration(subwindowConfiguration),
  m_windowViewportSize(windowViewportSize)
{
  // Reset the camera for each sub-window.
  for(size_t i = 0, subwindowCount = m_subwindowConfiguration->subwindow_count(); i < subwindowCount; ++i)
  {
    m_subwindowConfiguration->subwindow(i).reset_camera();
  }
}

//#################### DESTRUCTOR ####################

Renderer::~Renderer() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

ITMUChar4Image_CPtr Renderer::capture_screenshot() const
{
  // Read the pixel data from video memory into an image.
  const int width = m_windowViewportSize.width, height = m_windowViewportSize.height;
  ITMUChar4Image_Ptr screenshotImage(new ITMUChar4Image(Vector2i(width, height), true, false));
  Vector4u *pixelData = screenshotImage->GetData(MEMORYDEVICE_CPU);
  glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, pixelData);

  // Since the image we read from OpenGL will be upside-down, flip it before returning.
  for(int y = 0, halfHeight = height / 2; y < halfHeight; ++y)
  {
    int rowOffset1 = y * width, rowOffset2 = (height - 1 - y) * width;
    for(int x = 0; x < width; ++x)
    {
      std::swap(pixelData[rowOffset1 + x], pixelData[rowOffset2 + x]);
    }
  }

  return screenshotImage;
}

Vector2f Renderer::compute_fractional_window_position(int x, int y) const
{
  const Vector2f windowViewportSize = get_window_viewport_size().toFloat();
  return Vector2f(x / (windowViewportSize.x - 1), y / (windowViewportSize.y - 1));
}

bool Renderer::get_median_filtering_enabled() const
{
  return m_medianFilteringEnabled;
}

const SubwindowConfiguration_Ptr& Renderer::get_subwindow_configuration()
{
  return m_subwindowConfiguration;
}

SubwindowConfiguration_CPtr Renderer::get_subwindow_configuration() const
{
  return m_subwindowConfiguration;
}

void Renderer::set_median_filtering_enabled(bool medianFilteringEnabled)
{
  m_medianFilteringEnabled = medianFilteringEnabled;
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

Model_CPtr Renderer::get_model() const
{
  return m_model;
}

SDL_Window *Renderer::get_window() const
{
  return m_window.get();
}

const Vector2i& Renderer::get_window_viewport_size() const
{
  return m_windowViewportSize;
}

void Renderer::initialise_common()
{
  // Set up a texture in which to temporarily store scene visualisations and the touch image when rendering.
  glGenTextures(1, &m_textureID);
}

void Renderer::render_scene(const Vector2f& fracWindowPos, int viewIndex, const std::string& secondaryCameraName) const
{
  // Set the viewport for the window.
  const Vector2i& windowViewportSize = get_window_viewport_size();
  glViewport(0, 0, windowViewportSize.width, windowViewportSize.height);

  // Clear the frame buffer.
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Render all the sub-windows.
  for(size_t subwindowIndex = 0, count = m_subwindowConfiguration->subwindow_count(); subwindowIndex < count; ++subwindowIndex)
  {
    Subwindow& subwindow = m_subwindowConfiguration->subwindow(subwindowIndex);

    // If we have not yet started reconstruction for this sub-window's scene, skip rendering it.
    const std::string& sceneID = subwindow.get_scene_id();
    if(!m_model->get_slam_state(sceneID)->get_view()) continue;

    // Set the viewport for the sub-window.
    int left = (int)ROUND(subwindow.top_left().x * windowViewportSize.width);
    int top = (int)ROUND((1 - subwindow.bottom_right().y) * windowViewportSize.height);
    int width = (int)ROUND(subwindow.width() * windowViewportSize.width);
    int height = (int)ROUND(subwindow.height() * windowViewportSize.height);
    glViewport(left, top, width, height);

    // If the sub-window is in follow mode, update its camera.
    if(subwindow.get_camera_mode() == Subwindow::CM_FOLLOW)
    {
      ORUtils::SE3Pose livePose = m_model->get_slam_state(subwindow.get_scene_id())->get_pose();
      subwindow.get_camera()->set_from(CameraPoseConverter::pose_to_camera(livePose));
    }

    // Determine the pose from which to render.
    Camera_CPtr camera = secondaryCameraName == "" ? subwindow.get_camera() : subwindow.get_camera()->get_secondary_camera(secondaryCameraName);
    ORUtils::SE3Pose pose = CameraPoseConverter::camera_to_pose(*camera);

    // Render the reconstructed scene, then render a synthetic scene over the top of it.
    render_reconstructed_scene(sceneID, pose, subwindow.get_voxel_render_state(viewIndex), subwindow.get_surfel_render_state(viewIndex), subwindow);
    render_synthetic_scene(sceneID, pose);

#if WITH_GLUT && USE_PIXEL_DEBUGGING
    // Render the value of the pixel to which the user is pointing (for debugging purposes).
    render_pixel_value(fracWindowPos, subwindow);
#endif
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

void Renderer::generate_visualisation(const ITMUChar4Image_Ptr& output, const SpaintVoxelScene_CPtr& voxelScene, const SpaintSurfelScene_CPtr& surfelScene,
                                      VoxelRenderState_Ptr& voxelRenderState, SurfelRenderState_Ptr& surfelRenderState, const ORUtils::SE3Pose& pose, const View_CPtr& view,
                                      VisualisationGenerator::VisualisationType visualisationType, bool surfelFlag,
                                      const boost::optional<VisualisationGenerator::Postprocessor>& postprocessor) const
{
  VisualisationGenerator_CPtr visualisationGenerator = m_model->get_visualisation_generator();

  switch(visualisationType)
  {
    case VisualisationGenerator::VT_INPUT_COLOUR:
      visualisationGenerator->get_rgb_input(output, view);
      break;
    case VisualisationGenerator::VT_INPUT_DEPTH:
      visualisationGenerator->get_depth_input(output, view);
      break;
    default:
      if(surfelFlag) visualisationGenerator->generate_surfel_visualisation(output, surfelScene, pose, view, surfelRenderState, visualisationType);
      else visualisationGenerator->generate_voxel_visualisation(output, voxelScene, pose, view, voxelRenderState, visualisationType, postprocessor);
      break;
  }
}

void Renderer::render_overlay(const ITMUChar4Image_CPtr& overlay) const
{
  // Copy the overlay to a texture.
  glBindTexture(GL_TEXTURE_2D, m_textureID);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, overlay->noDims.x, overlay->noDims.y, 0, GL_RGBA, GL_UNSIGNED_BYTE, overlay->GetData(MEMORYDEVICE_CPU));
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

  // Enable blending.
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Render a semi-transparent quad textured with the overlay over the top of the existing scene.
  begin_2d();
    render_textured_quad(m_textureID);
  end_2d();

  // Disable blending again.
  glDisable(GL_BLEND);
}

#if WITH_GLUT && USE_PIXEL_DEBUGGING
void Renderer::render_pixel_value(const Vector2f& fracWindowPos, const Subwindow& subwindow) const
{
  boost::optional<std::pair<size_t,Vector2f> > fracSubwindowPos = m_subwindowConfiguration->compute_fractional_subwindow_position(fracWindowPos);
  if(!fracSubwindowPos) return;

  ITMUChar4Image_CPtr image = subwindow.get_image();
  int x = (int)ROUND(fracSubwindowPos->second.x * (image->noDims.x - 1));
  int y = (int)ROUND(fracSubwindowPos->second.y * (image->noDims.y - 1));
  Vector4u v = image->GetData(MEMORYDEVICE_CPU)[y * image->noDims.x + x];

  std::ostringstream oss;
  oss << x << ',' << y << ": " << (int)v.r << ',' << (int)v.g << ',' << (int)v.b << ',' << (int)v.a;

  begin_2d();
    render_text(oss.str(), Vector3f(1.0f, 0.0f, 0.0f), Vector2f(0.025f, 0.95f));
  end_2d();
}
#endif

void Renderer::render_reconstructed_scene(const std::string& sceneID, const SE3Pose& pose, VoxelRenderState_Ptr& voxelRenderState, SurfelRenderState_Ptr& surfelRenderState,
                                          Subwindow& subwindow) const
{
  // Set up any post-processing that needs to be applied to the rendering result.
  // FIXME: At present, median filtering breaks in CPU mode, so we prevent it from running, but we should investigate why.
  static boost::optional<VisualisationGenerator::Postprocessor> postprocessor = boost::none;
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

  // Generate the subwindow image.
  const ITMUChar4Image_Ptr& image = subwindow.get_image();
  SLAMState_CPtr slamState = m_model->get_slam_state(sceneID);
  generate_visualisation(
    image, slamState->get_voxel_scene(), slamState->get_surfel_scene(),
    voxelRenderState, surfelRenderState, pose, slamState->get_view(),
    subwindow.get_type(), subwindow.get_surfel_flag(), postprocessor
  );

  // Copy the raycasted scene to a texture.
  glBindTexture(GL_TEXTURE_2D, m_textureID);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image->noDims.x, image->noDims.y, 0, GL_RGBA, GL_UNSIGNED_BYTE, image->GetData(MEMORYDEVICE_CPU));
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

  // Render a quad textured with the subwindow image.
  begin_2d();
    render_textured_quad(m_textureID);
  end_2d();
}

void Renderer::render_synthetic_scene(const std::string& sceneID, const SE3Pose& pose) const
{
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_DEPTH_TEST);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  {
    SLAMState_CPtr slamState = m_model->get_slam_state(sceneID);
    ORUtils::Vector2<int> depthImageSize = slamState->get_depth_image_size();
    set_projection_matrix(slamState->get_intrinsics(), depthImageSize.width, depthImageSize.height);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    {
      // Note: Conveniently, data() returns the elements in column-major order (the order required by OpenGL).
      glLoadMatrixf(CameraPoseConverter::pose_to_modelview(pose).data());

      // Render the default camera.
      CameraRenderer::render_camera(*CameraFactory::make_default_camera());

      // Render the current selector to show how we're interacting with the scene.
      Vector3u labelColour = m_model->get_label_manager()->get_label_colour(m_model->get_semantic_label());
      Vector3f selectorColour(labelColour.r / 255.0f, labelColour.g / 255.0f, labelColour.b / 255.0f);
      SelectorRenderer selectorRenderer(this, selectorColour);
      SelectionTransformer_CPtr transformer = m_model->get_selection_transformer();
      if(transformer) transformer->accept(selectorRenderer);
      m_model->get_selector()->accept(selectorRenderer);
    }
    glPopMatrix();
  }
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();

  glDisable(GL_DEPTH_TEST);
}

#ifdef WITH_GLUT
void Renderer::render_text(const std::string& text, const Vector3f& colour, const Vector2f& pos, void *font)
{
  glColor3f(colour.r, colour.g, colour.b);
  glRasterPos2f(pos.x, pos.y);
  for(size_t i = 0, len = text.length(); i < len; ++i)
  {
    glutBitmapCharacter(font, text[i]);
  }
}
#endif

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
