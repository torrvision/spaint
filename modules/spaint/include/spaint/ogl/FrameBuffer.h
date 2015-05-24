/**
 * spaint: FrameBuffer.h
 */

#ifndef H_SPAINT_FRAMEBUFFER
#define H_SPAINT_FRAMEBUFFER

#include <boost/shared_ptr.hpp>

#include "WrappedGL.h"

namespace spaint {

/**
 * \brief An instance of this class represents an off-screen frame buffer to which OpenGL rendering calls can be directed.
 */
class FrameBuffer
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The texture ID of the frame buffer's colour buffer. */
  GLuint m_colourBufferID;

  /** The render buffer ID of the frame buffer's depth buffer. */
  GLuint m_depthBufferID;

  /** The ID of the frame buffer itself. */
  GLuint m_id;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a frame buffer with the specified dimensions.
   *
   * \param width   The width of the frame buffer.
   * \param height  The height of the frame buffer.
   */
  FrameBuffer(int width, int height);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the frame buffer.
   */
  ~FrameBuffer();

  //#################### COPY CONSTRUCTOR & ASSIGNMENT OPERATOR ####################
private:
  // Deliberately private and unimplemented.
  FrameBuffer(const FrameBuffer&);
  FrameBuffer& operator=(const FrameBuffer&);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the texture ID of the frame buffer's colour buffer.
   *
   * \return  The texture ID of the frame buffer's colour buffer.
   */
  GLuint get_colour_buffer_id() const;

  /**
   * \brief Gets the ID of the frame buffer itself.
   *
   * \return  The ID of the frame buffer itself.
   */
  GLuint get_id() const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<const FrameBuffer> FrameBuffer_CPtr;

}

#endif
