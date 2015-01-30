/**
 * tvgutil: ColourRGBA.cpp
 */

#include "colours/ColourRGBA.h"

namespace tvgutil {

//#################### CONSTRUCTORS ####################

ColourRGBA::ColourRGBA()
: m_a(), m_b(), m_g(), m_r()
{}

ColourRGBA::ColourRGBA(unsigned char r, unsigned char g, unsigned char b, unsigned char a)
: m_a(a), m_b(b), m_g(g), m_r(r)
{}

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

ColourRGBA ColourRGBA::from_chars(unsigned char r, unsigned char g, unsigned char b, unsigned char a)
{
  return ColourRGBA(r, g, b, a);
}

ColourRGBA ColourRGBA::from_ints(int r, int g, int b, int a)
{
  // Clamp the components to [0,255].
  if(r < 0) r = 0;
  else if(r > 255) r = 255;
  if(g < 0) g = 0;
  else if(g > 255) g = 255;
  if(b < 0) b = 0;
  else if(b > 255) b = 255;
  if(a < 0) a = 0;
  else if(a > 255) a = 255;

  return ColourRGBA(
    static_cast<unsigned char>(r),
    static_cast<unsigned char>(g),
    static_cast<unsigned char>(b),
    static_cast<unsigned char>(a)
  );
}

ColourRGBA ColourRGBA::from_floats(float r, float g, float b, float a)
{
  // Clamp the components to [0,1].
  if(r < 0.0f) r = 0.0f;
  else if(r > 1.0f) r = 1.0f;
  if(g < 0.0f) g = 0.0f;
  else if(g > 1.0f) g = 1.0f;
  if(b < 0.0f) b = 0.0f;
  else if(b > 1.0f) b = 1.0f;
  if(a < 0.0f) a = 0.0f;
  else if(a > 1.0f) a = 1.0f;

  return ColourRGBA(
    static_cast<unsigned char>(r * 255.0f + 0.5f),
    static_cast<unsigned char>(g * 255.0f + 0.5f),
    static_cast<unsigned char>(b * 255.0f + 0.5f),
    static_cast<unsigned char>(a * 255.0f + 0.5f)
  );
}

//#################### PUBLIC OPERATORS ####################

bool ColourRGBA::operator==(const ColourRGBA& rhs) const
{
  return m_r == rhs.m_r && m_g == rhs.m_g && m_b == rhs.m_b && m_a == rhs.m_a;
}

bool ColourRGBA::operator!=(const ColourRGBA& rhs) const
{
  return !(*this == rhs);
}

//#################### STREAM OPERATORS ####################

std::ostream& operator<<(std::ostream& os, const ColourRGBA& rhs)
{
  os << '['
     << static_cast<int>(rhs.r())
     << ','
     << static_cast<int>(rhs.g())
     << ','
     << static_cast<int>(rhs.b())
     << ','
     << static_cast<int>(rhs.a())
     << ']';
  return os;
}

}
