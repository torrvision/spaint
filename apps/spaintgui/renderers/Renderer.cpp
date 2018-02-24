/**
 * spaintgui: Renderer.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "Renderer.h"
using namespace ITMLib;
using namespace ORUtils;
using namespace rigging;

#include <itmx/geometry/GeometryUtil.h>
#include <itmx/util/CameraPoseConverter.h>
using namespace itmx;

#include <spaint/ogl/CameraRenderer.h>
#include <spaint/ogl/QuadricRenderer.h>
#include <spaint/selectiontransformers/interface/VoxelToCubeSelectionTransformer.h>
#include <spaint/selectors/PickingSelector.h>
#include <spaint/util/CameraFactory.h>
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
    // Render the camera representing the Leap Motion controller's coordinate frame.
    CameraRenderer::render_camera(selector.get_camera(), CameraRenderer::AXES_XYZ, 0.1f);

    // Get the most recent frame of data from the Leap Motion. If it's invalid or does not contain precisely one hand, early out.
    const Leap::Frame& frame = selector.get_frame();
    if(!frame.isValid() || frame.hands().count() != 1) return;

    // Render the virtual hand.
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
          selector.from_leap_position(bone.prevJoint()),
          selector.from_leap_position(bone.nextJoint()),
          LeapSelector::from_leap_size(bone.width() * 0.5f),
          LeapSelector::from_leap_size(bone.width() * 0.5f),
          10
        );

        glColor3f(1.0f, 0.0f, 0.0f);
        QuadricRenderer::render_sphere(selector.from_leap_position(bone.nextJoint()), LeapSelector::from_leap_size(bone.width() * 0.7f), 10, 10);
      }
    }

    // If the selector is in point mode and the user is pointing at a valid voxel in the world,
    // draw a line between the tip of the virtual index finger and the voxel in question.
    if(selector.get_mode() == LeapSelector::MODE_POINT && selector.get_position())
    {
      const Leap::Finger& indexFinger = hand.fingers()[1];
      Eigen::Vector3f start = selector.from_leap_position(indexFinger.tipPosition());
      Eigen::Vector3f end = *selector.get_position();

      glColor3f(0.0f, 1.0f, 1.0f);
      glBegin(GL_LINES);
        glVertex3f(start.x(), start.y(), start.z());
        glVertex3f(end.x(), end.y(), end.z());
      glEnd();
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
: m_medianFilteringEnabled(false),
  m_model(model),
  m_subwindowConfiguration(subwindowConfiguration),
  m_supersamplingEnabled(false),
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

bool Renderer::get_supersampling_enabled() const
{
  return m_supersamplingEnabled;
}

void Renderer::render_client_images() const
{
  MappingServer_CPtr mappingServer = m_model->get_mapping_server();
  if(!mappingServer) return;

  std::vector<int> clients = mappingServer->get_active_clients();
  for(size_t i = 0, size = clients.size(); i < size; ++i)
  {
    MappingServer::RenderingImageHandler_Ptr imageHandler = mappingServer->get_rendering_image(clients[i]);
    if(!imageHandler) continue;

    const boost::optional<ORUtils::SE3Pose>& optionalPose = mappingServer->get_rendering_pose(clients[i]);
    if(!optionalPose) continue;

    ITMUChar4Image_Ptr& image = imageHandler->get();
    if(!image)
    {
      // FIXME: The size shouldn't be hard-coded like this.
      image.reset(new ITMUChar4Image(Vector2i(640,480), true, true));
    }

    ORUtils::SE3Pose pose = *optionalPose;

    // FIXME: The primary scene ID shouldn't be hard-coded like this.
    const std::string primarySceneID = Model::get_world_scene_id();

    static VoxelRenderState_Ptr voxelRenderState;
    static SurfelRenderState_Ptr surfelRenderState;
    const bool surfelFlag = false;
    render_all_reconstructed_scenes(pose, primarySceneID, VisualisationGenerator::VT_SCENE_COLOUR, voxelRenderState, surfelRenderState, image->noDims, surfelFlag, image);
  }
}

void Renderer::set_median_filtering_enabled(bool medianFilteringEnabled)
{
  m_medianFilteringEnabled = medianFilteringEnabled;
}

void Renderer::set_supersampling_enabled(bool supersamplingEnabled)
{
  m_supersamplingEnabled = supersamplingEnabled;

  for(size_t i = 0, subwindowCount = m_subwindowConfiguration->subwindow_count(); i < subwindowCount; ++i)
  {
    Subwindow& subwindow = m_subwindowConfiguration->subwindow(i);
    subwindow.resize_image(supersamplingEnabled ? Vector2i(1280, 960) : subwindow.get_original_image_size());
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
  glDeleteTextures(1, &m_textureID);
}

void Renderer::end_2d()
{
  glDepthMask(true);

  glMatrixMode(GL_MODELVIEW);
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

void Renderer::render_scene(const Vector2f& fracWindowPos, bool renderFiducials, int viewIndex, const std::string& secondaryCameraName) const
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
    SLAMState_CPtr slamState = m_model->get_slam_state(sceneID);
    if(!slamState || !slamState->get_view()) continue;

    // Set the viewport for the sub-window.
    int left = (int)ROUND(subwindow.top_left().x * windowViewportSize.width);
    int top = (int)ROUND((1 - subwindow.bottom_right().y) * windowViewportSize.height);
    int width = (int)ROUND(subwindow.width() * windowViewportSize.width);
    int height = (int)ROUND(subwindow.height() * windowViewportSize.height);
    glViewport(left, top, width, height);

    MappingClient_CPtr mappingClient = m_model->get_mapping_client(sceneID);
    if(mappingClient && subwindow.get_remote_flag())
    {
      // TODO: Comment here.
      ITMUChar4Image_CPtr remoteImage = mappingClient->get_remote_image();
      render_image(remoteImage);
    }
    else
    {
      // If the sub-window is in follow mode, update its camera.
      if(subwindow.get_camera_mode() == Subwindow::CM_FOLLOW)
      {
        ORUtils::SE3Pose livePose = slamState->get_pose();
        subwindow.get_camera()->set_from(CameraPoseConverter::pose_to_camera(livePose));
      }

      // Determine the pose from which to render.
      Camera_CPtr camera = secondaryCameraName == "" ? subwindow.get_camera() : subwindow.get_camera()->get_secondary_camera(secondaryCameraName);
      ORUtils::SE3Pose pose = CameraPoseConverter::camera_to_pose(*camera);

      if(subwindow.get_all_scenes_flag())
      {
        // Render all the reconstructed scenes in the same subwindow, with appropriate depth testing.
        render_all_reconstructed_scenes(pose, subwindow, viewIndex);
      }
      else
      {
        // Render the reconstructed scene.
        render_reconstructed_scene(sceneID, pose, subwindow, viewIndex);
      }

      // Render the synthetic scene over the top of the reconstructed scene.
      render_synthetic_scene(sceneID, pose, subwindow.get_camera_mode(), renderFiducials);
    }

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
                                      const ITMIntrinsics& intrinsics, VisualisationGenerator::VisualisationType visualisationType, bool surfelFlag) const
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
    {
      if(view)
      {
        if(surfelFlag) visualisationGenerator->generate_surfel_visualisation(output, surfelScene, pose, intrinsics, surfelRenderState, visualisationType);
        else visualisationGenerator->generate_voxel_visualisation(output, voxelScene, pose, intrinsics, voxelRenderState, visualisationType, get_postprocessor());
      }
      else output->Clear();

      break;
    }
  }
}

const boost::optional<VisualisationGenerator::Postprocessor>& Renderer::get_postprocessor() const
{
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
  return postprocessor;
}

void Renderer::render_all_reconstructed_scenes(const ORUtils::SE3Pose& primaryPose, const std::string& primarySceneID,
                                               VisualisationGenerator::VisualisationType primaryVisualisationType,
                                               VoxelRenderState_Ptr& voxelRenderState, SurfelRenderState_Ptr& surfelRenderState,
                                               const Vector2i& originalImgSize, bool surfelFlag, const ITMUChar4Image_Ptr& output) const
{
  static std::vector<ITMUChar4Image_Ptr> colourImages;
  static std::vector<ITMFloatImage_Ptr> depthImages;
  static bool supersamplingEnabled = m_supersamplingEnabled;
  const std::vector<std::string> sceneIDs = m_model->get_scene_ids();
  std::vector<VisualisationGenerator::VisualisationType> visualisationTypes(sceneIDs.size());

  // Step 1: If supersampling has been toggled since the last time we rendered the scenes, arrange for the colour and depth images for the scenes to be reallocated.
  if(supersamplingEnabled != m_supersamplingEnabled)
  {
    supersamplingEnabled = m_supersamplingEnabled;
    colourImages.clear();
    depthImages.clear();
  }

  // Step 2: Reallocate the colour and depth images for the scenes if needed.
  while(colourImages.size() < sceneIDs.size())
  {
    colourImages.push_back(ITMUChar4Image_Ptr(new ITMUChar4Image(output->noDims, true, true)));
    depthImages.push_back(ITMFloatImage_Ptr(new ITMFloatImage(output->noDims, true, true)));
  }

  // Step 3: Render colour and depth images for each scene, making sure to render the primary scene for the
  //         subwindow last so that the raycast result ultimately contains the correct voxels for picking.
  size_t primarySceneIdx = 0;
  for(size_t i = 0; i < sceneIDs.size() + 1; ++i)
  {
    // Determine which scene we should be rendering in this iteration of the loop. This will be the scene
    // indicated by the loop counter unless either (i) i < sceneIDs[i] && sceneIDs[i] == primarySceneID,
    // in which case we record the array index of the primary scene for later and skip the loop iteration,
    // or (ii) i == sceneIDs.size(), in which case we arrange for the primary scene to be rendered.
    size_t sceneIdx;
    if(i < sceneIDs.size())
    {
      if(sceneIDs[i] == primarySceneID)
      {
        primarySceneIdx = i;
        continue;
      }
      else sceneIdx = i;
    }
    else sceneIdx = primarySceneIdx;

    // Clear the colour and depth images for the chosen scene.
    colourImages[sceneIdx]->Clear();
    depthImages[sceneIdx]->Clear();

    // If we have not yet started reconstruction for this scene, avoid rendering it and early out.
    SLAMState_CPtr slamState = m_model->get_slam_state(sceneIDs[sceneIdx]);
    if(!slamState || !slamState->get_view()) continue;

    // Determine the pose and visualisation type to use for the scene.
    SE3Pose tempPose = primaryPose;
    visualisationTypes[sceneIdx] = primaryVisualisationType;

    if(sceneIDs[sceneIdx] != primarySceneID)
    {
      boost::optional<std::pair<SE3Pose,size_t> > result = m_model->get_collaborative_pose_optimiser()->try_get_relative_transform(primarySceneID, sceneIDs[sceneIdx]);
      SE3Pose relativeTransform = result ? result->first : SE3Pose(static_cast<float>((sceneIdx + 1) * 2.0f), 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      if(!result || result->second < CollaborativePoseOptimiser::confidence_threshold()) visualisationTypes[sceneIdx] = VisualisationGenerator::VT_SCENE_SEMANTICPHONG;

      // ciTwi * wiTwj = ciTwj
      tempPose.SetM(tempPose.GetM() * relativeTransform.GetM());
    }

    // Actually render the colour and depth images for the scene.
    SLAMState_CPtr primarySlamState = m_model->get_slam_state(primarySceneID);
    const SLAMState_CPtr& bestSlamState = primarySlamState && primarySlamState->get_view() ? primarySlamState : slamState;
    const ITMIntrinsics intrinsics = bestSlamState->get_view()->calib.intrinsics_d.MakeRescaled(originalImgSize, output->noDims);

    generate_visualisation(
      colourImages[sceneIdx], slamState->get_voxel_scene(), slamState->get_surfel_scene(),
      voxelRenderState, surfelRenderState, tempPose, slamState->get_view(), intrinsics,
      visualisationTypes[sceneIdx], surfelFlag
    );

    m_model->get_visualisation_generator()->generate_depth_from_voxels(
      depthImages[sceneIdx], slamState->get_voxel_scene(), tempPose, intrinsics,
      voxelRenderState, DepthVisualiser::DT_ORTHOGRAPHIC
    );

    // Make sure the depth image for the scene is available on the CPU so that it can be used for depth testing.
    depthImages[sceneIdx]->UpdateHostFromDevice();
  }

  // Step 4: Combine the colour images for the different scenes using per-pixel depth testing to produce the final output image.
  output->Clear();
  for(int k = 0; k < output->noDims.width * output->noDims.height; ++k)
  {
    float smallestDepth = static_cast<float>(INT_MAX);
    for(size_t i = 0, size = colourImages.size(); i < size; ++i)
    {
      const float arbitrarilyLargeDepth = 100.0f;
      float depth = depthImages[i]->GetData(MEMORYDEVICE_CPU)[k];
      if(depth != -1.0f && visualisationTypes[i] == VisualisationGenerator::VT_SCENE_SEMANTICPHONG) depth = arbitrarilyLargeDepth;
      if(depth != -1.0f && depth < smallestDepth)
      {
        smallestDepth = depth;
        output->GetData(MEMORYDEVICE_CPU)[k] = colourImages[i]->GetData(MEMORYDEVICE_CPU)[k];
      }
    }
  }

  // Step 5: Render a quad textured with the final output image.
  render_image(output);
}

void Renderer::render_all_reconstructed_scenes(const ORUtils::SE3Pose& primaryPose, Subwindow& subwindow, int viewIndex) const
{
  render_all_reconstructed_scenes(
    primaryPose, subwindow.get_scene_id(), subwindow.get_type(), subwindow.get_voxel_render_state(viewIndex), subwindow.get_surfel_render_state(viewIndex),
    subwindow.get_original_image_size(), subwindow.get_surfel_flag(), subwindow.get_image()
  );
}

void Renderer::render_image(const ITMUChar4Image_CPtr& image, bool useAlphaBlending) const
{
  // Copy the image to a texture.
  glBindTexture(GL_TEXTURE_2D, m_textureID);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image->noDims.x, image->noDims.y, 0, GL_RGBA, GL_UNSIGNED_BYTE, image->GetData(MEMORYDEVICE_CPU));
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  if(useAlphaBlending)
  {
    // Enable alpha blending.
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  }

  // Render a quad textured with the image over the top of the existing scene.
  begin_2d();
    render_textured_quad(m_textureID);
  end_2d();

  if(useAlphaBlending)
  {
    // Disable alpha blending again.
    glDisable(GL_BLEND);
  }
}

void Renderer::render_overlay(const ITMUChar4Image_CPtr& overlay) const
{
  const bool useAlphaBlending = true;
  render_image(overlay, useAlphaBlending);
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

void Renderer::render_reconstructed_scene(const std::string& sceneID, const SE3Pose& pose, Subwindow& subwindow, int viewIndex) const
{
  // Generate the subwindow image.
  const ITMUChar4Image_Ptr& image = subwindow.get_image();

  SLAMState_CPtr slamState = m_model->get_slam_state(sceneID);
  const View_CPtr view = slamState->get_view();
  const ITMIntrinsics intrinsics = view->calib.intrinsics_d.MakeRescaled(subwindow.get_original_image_size(), image->noDims);

  generate_visualisation(
    image, slamState->get_voxel_scene(), slamState->get_surfel_scene(),
    subwindow.get_voxel_render_state(viewIndex), subwindow.get_surfel_render_state(viewIndex),
    pose, view, intrinsics, subwindow.get_type(), subwindow.get_surfel_flag()
  );

  // Render a quad textured with the subwindow image.
  render_image(image);
}

void Renderer::render_synthetic_scene(const std::string& sceneID, const SE3Pose& pose, Subwindow::CameraMode cameraMode, bool renderFiducials) const
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
      static SimpleCamera defaultCam = *CameraFactory::make_default_camera();
      CameraRenderer::render_camera(defaultCam);

      // Render the current selector to show how we're interacting with the scene.
      Vector3u labelColour = m_model->get_label_manager()->get_label_colour(m_model->get_semantic_label());
      Vector3f selectorColour(labelColour.r / 255.0f, labelColour.g / 255.0f, labelColour.b / 255.0f);
      SelectorRenderer selectorRenderer(this, selectorColour);
      SelectionTransformer_CPtr transformer = m_model->get_selection_transformer();
      if(transformer) transformer->accept(selectorRenderer);
      m_model->get_selector()->accept(selectorRenderer);

      // If we're rendering fiducials, render any that have been detected.
      if(renderFiducials)
      {
        const std::map<std::string,Fiducial_Ptr>& fiducials = slamState->get_fiducials();
        for(std::map<std::string,Fiducial_Ptr>::const_iterator it = fiducials.begin(), iend = fiducials.end(); it != iend; ++it)
        {
          float confidence = it->second->confidence();
          if(confidence < Fiducial::stable_confidence()) continue;

          SimpleCamera cam = CameraPoseConverter::pose_to_camera(it->second->pose());
          float c = CLAMP(confidence / Fiducial::stable_confidence(), 0.0f, 1.0f);
          CameraRenderer::render_camera(cam, CameraRenderer::AXES_XYZ, 0.1f, Vector3f(c, c, 0.0f));
        }
      }

      // If the camera for the subwindow is in follow mode, render any overlay image generated during object segmentation.
      if(cameraMode == Subwindow::CM_FOLLOW)
      {
        const ITMUChar4Image_CPtr& segmentationImage = m_model->get_segmentation_image(sceneID);
        if(segmentationImage) render_overlay(segmentationImage);
      }
    }
    glMatrixMode(GL_MODELVIEW);
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
