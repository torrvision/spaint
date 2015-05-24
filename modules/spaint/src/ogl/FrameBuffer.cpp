/**
 * spaint: FrameBuffer.cpp
 */

#include "ogl/FrameBuffer.h"

#include <cstddef>

namespace spaint {

//#################### CONSTRUCTORS ####################

FrameBuffer::FrameBuffer(int width, int height)
{
  // Set up the colour buffer.
  glGenTextures(1, &m_colourBufferID);
  glBindTexture(GL_TEXTURE_2D, m_colourBufferID);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

  // Set up the depth buffer.
  glGenRenderbuffers(1, &m_depthBufferID);
  glBindRenderbuffer(GL_RENDERBUFFER, m_depthBufferID);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height);

  // Set up the frame buffer.
  glGenFramebuffers(1, &m_id);
  glBindFramebuffer(GL_FRAMEBUFFER, m_id);

  // Attach the colour buffer to the frame buffer.
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_colourBufferID, 0);
  GLenum drawBuffer = GL_COLOR_ATTACHMENT0;
  glDrawBuffers(1, &drawBuffer);

  // Attach the depth buffer to the frame buffer.
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_depthBufferID);

  // Switch back to rendering to the normal display.
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

//#################### DESTRUCTOR ####################

FrameBuffer::~FrameBuffer()
{
  glDeleteRenderbuffers(1, &m_depthBufferID);
  glDeleteTextures(1, &m_colourBufferID);
  glDeleteFramebuffers(1, &m_id);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

GLuint FrameBuffer::get_colour_buffer_id() const
{
  return m_colourBufferID;
}

GLuint FrameBuffer::get_id() const
{
  return m_id;
}

}
