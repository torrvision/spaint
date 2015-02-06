/**
 * tvgutil: PaletteGenerator.h
 */

#ifndef H_TVGUTIL_PALETTEGENERATOR
#define H_TVGUTIL_PALETTEGENERATOR

#include <map>
#include <set>

#include <boost/assign/list_of.hpp>

#include "../RandomNumberGenerator.h"

namespace tvgutil {

/**
 * \brief This class provides functions that can be used to generate palettes (maps from labels to colours).
 */
class PaletteGenerator
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Constructs a palette with a random RGBA colour per label.
   *
   * \param labels  The set of labels.
   * \param seed    The seed for the random number generator.
   * \return        The generated palette.
   */
  template <typename Label, typename Colour>
  static std::map<Label,Colour> generate_random_rgba_palette(const std::set<Label>& labels, unsigned int seed)
  {
    std::map<Label,Colour> result;
    RandomNumberGenerator rng(seed);

    for(typename std::set<Label>::const_iterator it = labels.begin(), iend = labels.end(); it != iend; ++it)
    {
      int r = rng.generate_int_from_uniform(0, 255);
      int g = rng.generate_int_from_uniform(0, 255);
      int b = rng.generate_int_from_uniform(0, 255);
      result.insert(std::make_pair(*it, Colour(r, g, b, 255)));
    }

    return result;
  }

  /**
   * \brief Constructs a palette with commonly used colours.
   *
   * \return The generated palette.
   */
  template <typename Colour>
  static std::map<std::string,Colour> generate_basic_rgba_palette()
  {
    const int alpha = 255;

    std::map<std::string,Colour> result = boost::assign::map_list_of
      ("Black",Colour(0,0,0,alpha))
      ("White",Colour(255,255,255,alpha))
      ("Red",Colour(255,0,0,alpha))
      ("Lime",Colour(0,255,0,alpha))
      ("Blue",Colour(0,0,255,alpha))
      ("Yellow",Colour(255,255,0,alpha))
      ("Cyan",Colour(0,255,255,alpha))
      ("Magneta",Colour(255,0,255,alpha))
      ("Silver",Colour(192,192,192,alpha))
      ("Gray",Colour(128,128,128,alpha))
      ("Maroon",Colour(128,0,0,alpha))
      ("Olive",Colour(128,128,0,alpha))
      ("Green",Colour(0,128,0,alpha))
      ("Purple",Colour(128,0,128,alpha))
      ("Teal",Colour(0,128,128,alpha))
      ("Navy",Colour(0,0,128,alpha));

    return result;
  }
};

}

#endif
