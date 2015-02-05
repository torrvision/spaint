/**
 * tvgutil: PaletteGenerator.h
 */

#ifndef H_TVGUTIL_PALETTEGENERATOR
#define H_TVGUTIL_PALETTEGENERATOR

#include <map>
#include <set>

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

};

}

#endif
