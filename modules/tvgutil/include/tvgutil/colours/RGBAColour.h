/**
 * tvgutil: RGBAColour.h
 */

#ifndef H_TVGUTIL_RGBACOLOUR
#define H_TVGUTIL_RGBACOLOUR

#include <ostream>

namespace tvgutil {

/**
 * \brief An instance of this class represents an RGBA (red/green/blue/alpha) colour.
 *
 * The RGBA values represent possible states an image pixel may assume.
 */
template <typename T>
class RGBAColour
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The alpha value. */
  T m_a;

  /** The blue value. */
  T m_b;

  /** The green value. */
  T m_g;

  /** The red value. */
  T m_r;

  //#################### CONSTRUCTOR ####################
public:
  /**
   * \brief Constructs a default RGBA colour, with components that are all set to zero.
   */
  RGBAColour()
  : m_r(), m_g(), m_b(), m_a()
  {}

  /**
   * \brief Constructs an RGBA colour.
   *
   * \param r The red value.
   * \param g The green value.
   * \param b The blue value.
   * \param a The alpha value.
   */
  RGBAColour(T r, T g, T b, T a)
  : m_r(r), m_g(g), m_b(b), m_a(a)
  {}

  //#################### PUBLIC OPERATORS ####################
public:
  /**
   * \brief Determines whether this colour equals another one.
   *
   * \param rhs The other colour.
   * \return    true, if the two colours are equal, or false otherwise.
   */
  bool operator==(const RGBAColour& rhs) const
  {
    return m_r == rhs.m_r && m_g == rhs.m_g && m_b == rhs.m_b && m_a == rhs.m_a;
  }

  /**
   * \brief Determines whether this colour does not equal another one.
   *
   * \param rhs The other colour.
   * \return    true, if the two colours are different, or false otherwise.
   */
  bool operator!=(const RGBAColour& rhs) const
  {
    return !(*this == rhs);
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the alpha value.
   *
   * \return The alpha value.
   */
  T a() const
  {
    return m_a;
  }

  /**
   * \brief Gets the blue value.
   *
   * \return The blue value.
   */
  T b() const
  {
    return m_b;
  }

  /**
   * \brief Gets the green value.
   *
   * \return The green value.
   */
  T g() const
  {
    return m_g;
  }

  /**
   * \brief Gets the red value.
   *
   * \return The red value.
   */
  T r() const
  {
    return m_r;
  }
};

//#################### STREAM OPERATORS ####################

/**
 * \brief Outputs a colour to a stream.
 *
 * \param os  The stream.
 * \param rhs The colour.
 * \return    The stream.
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, const RGBAColour<T>& rhs)
{
  os << '[' << rhs.r() << ',' << rhs.g() << ',' << rhs.b() << ',' << rhs.a() << ']';
  return os;
}

/**
 * \brief Outputs a colour whose components are unsigned chars to a stream.
 *
 * \param os  The stream.
 * \param rhs The colour.
 * \return    The stream.
 */
inline std::ostream& operator<<(std::ostream& os, const RGBAColour<unsigned char>& rhs)
{
  os << '[' << static_cast<int>(rhs.r()) << ',' << static_cast<int>(rhs.g()) << ',' << static_cast<int>(rhs.b()) << ',' << static_cast<int>(rhs.a()) << ']';
  return os;
}

//#################### TYPEDEFS ####################

typedef RGBAColour<double> RGBAColourD;
typedef RGBAColour<float> RGBAColourF;
typedef RGBAColour<int> RGBAColourI;
typedef RGBAColour<unsigned char> RGBAColourUC;

}

#endif
