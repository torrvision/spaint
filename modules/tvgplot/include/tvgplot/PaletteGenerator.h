/**
 * tvgplot: PaletteGenerator.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_TVGPLOT_PALETTEGENERATOR
#define H_TVGPLOT_PALETTEGENERATOR

#include <map>
#include <set>

#include <boost/assign/list_of.hpp>

#include <opencv2/core/core.hpp>

#include <tvgutil/numbers/RandomNumberGenerator.h>

namespace tvgplot {

/**
 * \brief This class provides functions that can be used to generate palettes (maps from labels to colours).
 */
class PaletteGenerator
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Constructs a palette with commonly-used colours.
   *
   * \return The generated palette.
   */
  static std::map<std::string,cv::Scalar> generate_basic_rgba_palette();

  /**
   * \brief Constructs a palette with a random RGBA colour per label.
   *
   * \param labels  The set of labels.
   * \param seed    The seed for the random number generator.
   * \return        The generated palette.
   */
  template <typename Label>
  static std::map<Label,cv::Scalar> generate_random_rgba_palette(const std::set<Label>& labels, unsigned int seed)
  {
    std::map<Label,cv::Scalar> result;
    tvgutil::RandomNumberGenerator rng(seed);

    for(typename std::set<Label>::const_iterator it = labels.begin(), iend = labels.end(); it != iend; ++it)
    {
      int r = rng.generate_int_from_uniform(0, 255);
      int g = rng.generate_int_from_uniform(0, 255);
      int b = rng.generate_int_from_uniform(0, 255);
      result.insert(std::make_pair(*it, cv::Scalar(r, g, b, 255)));
    }

    return result;
  }
};

}

#endif
