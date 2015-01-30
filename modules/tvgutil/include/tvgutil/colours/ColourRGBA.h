/**
 * tvgutil: ColourRGBA.h
 */

#ifndef H_TVGUTIL_COLOURRGBA
#define H_TVGUTIL_COLOURRGBA

#include <ostream>
#include <stdexcept>

namespace tvgutil {

/**
 * \brief An instance of this class represents an RGBA (red/green/blue/alpha) colour.
 */
class ColourRGBA
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The alpha component (0-255). */
  unsigned char m_a;

  /** The blue component (0-255). */
  unsigned char m_b;

  /** The green component (0-255). */
  unsigned char m_g;

  /** The red component (0-255). */
  unsigned char m_r;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a default RGBA colour, with components that are all set to zero.
   */
  ColourRGBA();

private:
  /**
   * \brief Constructs an RGBA colour.
   *
   * \param r The red component (0-255).
   * \param g The green component (0-255).
   * \param b The blue component (0-255).
   * \param a The alpha component (0-255).
   */
  ColourRGBA(unsigned char r, unsigned char g, unsigned char b, unsigned char a);

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Constructs an RGBA colour from components expressed as unsigned chars.
   *
   * \param r The red component (0-255).
   * \param g The green component (0-255).
   * \param b The blue component (0-255).
   * \param a The alpha component (0-255).
   */
  static ColourRGBA from_chars(unsigned char r, unsigned char g, unsigned char b, unsigned char a);

  /**
   * \brief Constructs an RGBA colour from components expressed as ints.
   *
   * Note that the supplied components will be clamped to the range [0,255].
   *
   * \param r The red component (0-255).
   * \param g The green component (0-255).
   * \param b The blue component (0-255).
   * \param a The alpha component (0-255).
   */
  static ColourRGBA from_ints(int r, int g, int b, int a);

  /**
   * \brief Constructs an RGBA colour from components expressed as floats.
   *
   * Note that the supplied components will be clamped to the range [0,1].
   *
   * \param r The red component (0.0-1.0).
   * \param g The green component (0.0-1.0).
   * \param b The blue component (0.0-1.0).
   * \param a The alpha component (0.0-1.0).
   */
  static ColourRGBA from_floats(float r, float g, float b, float a);

  //#################### PUBLIC OPERATORS ####################
public:
  /**
   * \brief Checks whether this colour equals another one.
   *
   * \param rhs The other colour.
   * \return    true, if the two colours are equal, or false otherwise.
   */
  bool operator==(const ColourRGBA& rhs) const;

  /**
   * \brief Checks whether this colour differs from another one.
   *
   * \param rhs The other colour.
   * \return    true, if the two colours are different, or false otherwise.
   */
  bool operator!=(const ColourRGBA& rhs) const;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the alpha component.
   *
   * \return The alpha component.
   */
  inline unsigned char a() const
  {
    return m_a;
  }

  /**
   * \brief Gets the blue component.
   *
   * \return The blue component.
   */
  inline unsigned char b() const
  {
    return m_b;
  }

  /**
   * \brief Gets the green component.
   *
   * \return The green component.
   */
  inline unsigned char g() const
  {
    return m_g;
  }

  /**
   * \brief Gets the red component.
   *
   * \return The red component.
   */
  inline unsigned char r() const
  {
    return m_r;
  }
};

//#################### STREAM OPERATORS ####################

/**
 * \brief Outputs an RGBA colour to a stream.
 *
 * \param os  The stream.
 * \param rhs The colour.
 * \return    The stream.
 */
std::ostream& operator<<(std::ostream& os, const ColourRGBA& rhs);

}

#endif
