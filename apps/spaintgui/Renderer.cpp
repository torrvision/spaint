/**
 * spaintgui: Renderer.cpp
 */

#include "Renderer.h"

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

void Renderer::render_to_buffer(const ITMPose& pose, const spaint::SpaintInteractor_CPtr& interactor, GLuint frameBufferID)
{
  // TODO
}
