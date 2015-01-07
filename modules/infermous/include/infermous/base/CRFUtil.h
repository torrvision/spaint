/**
 * infermous: CRFUtil.h
 */

#ifndef H_INFERMOUS_CRFUTIL
#define H_INFERMOUS_CRFUTIL

#include <tvgutil/ArgUtil.h>

#include "Grids.h"

namespace infermous {

/**
 * \brief This class contains various CRF utility functions.
 */
struct CRFUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Calculates the pixels that fall within a circle of the specified radius, centred at the origin (excluding the origin itself).
   *
   * The intention is to define a set of offsets that can be used to specify the neighbours of an arbitrary pixel.
   * The origin itself is excluded from this set, because a point cannot be its own neighbour.
   *
   * \param radius  The radius of the circle.
   * \return        The calculated set of offsets.
   */
  static std::vector<Eigen::Vector2i> make_circular_neighbour_offsets(int radius)
  {
    std::vector<Eigen::Vector2i> result;

    float radiusSquared = static_cast<float>(radius * radius);
    for(int y = -radius; y <= radius; ++y)
    {
      for(int x = -radius; x <= radius; ++x)
      {
        if(x == 0 && y == 0) continue;

        float distanceSquared = static_cast<float>(x*x + y*y);
        if(distanceSquared <= radiusSquared) result.push_back(Eigen::Vector2i(x, y));
      }
    }

    return result;
  }

  /**
   * \brief Calculates the pixels that fall within a square of the specified axial radius, centred at the origin (excluding the origin itself).
   *
   * The intention is to define a set of offsets that can be used to specify the neighbours of an arbitrary pixel.
   * The origin itself is excluded from this set, because a point cannot be its own neighbour.
   *
   * \param radius  The axial radius of the square.
   * \return        The calculated set of offsets.
   */
  static std::vector<Eigen::Vector2i> make_square_neighbour_offsets(int radius)
  {
    std::vector<Eigen::Vector2i> result;

    for(int y = -radius; y <= radius; ++y)
    {
      for(int x = -radius; x <= radius; ++x)
      {
        if(x != 0 || y != 0) result.push_back(Eigen::Vector2i(x, y));
      }
    }

    return result;
  }

  /**
   * \brief Predicts the labels for each pixel in a probabilities grid by choosing a label with the highest probability for each pixel.
   *
   * \param probabilities The grid of probabilities for whose pixels we want to predict labels.
   * return               The grid of predicted labels.
   */
  template <typename Label>
  static Grid<Label> predict_labels(const ProbabilitiesGrid<Label>& probabilities)
  {
    Grid<Label> result(probabilities.cols(), probabilities.rows());
    for(size_t y = 0, height = probabilities.rows(); y < height; ++y)
    {
      for(size_t x = 0, width = probabilities.cols(); x < width; ++x)
      {
        result(x, y) = tvgutil::ArgUtil::argmax(probabilities(x, y));
      }
    }
    return result;
  }
};

}

#endif
