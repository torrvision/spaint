/**
 * spaint: FrameBuffer.cpp
 */

#include "ogl/FrameBuffer.h"

#include <cstddef>
#include <stdexcept>

namespace spaint {

//#################### CONSTRUCTORS ####################

FrameBuffer::FrameBuffer(int width, int height)
{
  // Set up the colour buffer.
  glGenTextures(1, &m_colourBufferID);
  glBindTexture(GL_TEXTURE_2D, m_colourBufferID);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

  // Set up the depth buffer.
  glGenRenderbuffers(1, &m_depthBufferID);
  glBindRenderbuffer(GL_RENDERBUFFER, m_depthBufferID);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);

  // Set up the frame buffer.
  glGenFramebuffers(1, &m_id);
  glBindFramebuffer(GL_FRAMEBUFFER, m_id);

  // Attach the colour buffer to the frame buffer.
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_colourBufferID, 0);

  // Attach the depth buffer to the frame buffer.
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_depthBufferID);

  // Check that the frame buffer has been successfully set up.
  if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
  {
    throw std::runtime_error("Failed to set up the frame buffer");
  }

  // Switch back to rendering to the normal display.
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

//#################### DESTRUCTOR ####################

FrameBuffer::~FrameBuffer()
{
#if 0
  glBindFramebuffer(GL_FRAMEBUFFER, m_id);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
#endif

#if 0
  glDeleteRenderbuffers(1, &m_depthBufferID);
  glDeleteTextures(1, &m_colourBufferID);  
  glDeleteFramebuffers(1, &m_id);
#endif
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
