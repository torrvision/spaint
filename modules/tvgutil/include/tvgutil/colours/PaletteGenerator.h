/**
 * tvgutil: PaletteGenerator.h
 */

#ifndef H_TVGUTIL_PALETTEGENERATOR
#define H_TVGUTIL_PALETTEGENERATOR

#include <map>
#include <set>

#include "../RandomNumberGenerator.h"

#include "RGBAColour.h"

namespace tvgutil {

/**
 * \brief This class provides functions that can be used to generate palettes (maps from labels to colours).
 */
class PaletteGenerator
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Constructs a palette with a random colour per label.
   *
   * \param labels  The set of labels.
   * \param seed    The seed for the random number generator.
   */
  template <typename Label>
  static std::map<Label,RGBAColourUC> generate_random_palette(const std::set<Label>& labels, unsigned int seed)
  {
    std::map<Label,RGBAColourUC> result;
    RandomNumberGenerator rng(seed);

    for(typename std::set<Label>::const_iterator it = labels.begin(), iend = labels.end(); it != iend; ++it)
    {
      result.insert(std::make_pair(*it, RGBAColourUC(
        static_cast<unsigned char>(rng.generate_int_from_uniform(0, 255)),
        static_cast<unsigned char>(rng.generate_int_from_uniform(0, 255)),
        static_cast<unsigned char>(rng.generate_int_from_uniform(0, 255)),
        static_cast<unsigned char>(255)
      )));
    }

    return result;
  }
};

}

#endif
