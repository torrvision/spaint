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
   * \brief TODO
   *
   * \param radius  TODO
   * \return        TODO
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
   * \brief TODO
   *
   * \param radius  TODO
   * \return        TODO
   */
  static std::vector<Eigen::Vector2i> make_square_neighbour_offsets(int radius)
  {
    // TODO
    throw 23;
  }

  /**
   * \brief Predicts the labels for each pixel in a potentials grid by choosing a label with the highest potential for each pixel.
   *
   * \param grid  The grid of potentials for whose pixels we want to predict labels.
   * return       The grid of predicted labels.
   */
  template <typename Label>
  static Grid<Label> predict_labels(const PotentialsGrid<Label>& potentials)
  {
    Grid<Label> result(potentials.cols(), potentials.rows());
    for(size_t y = 0, height = potentials.rows(); y < height; ++y)
    {
      for(size_t x = 0, width = potentials.cols(); x < width; ++x)
      {
        result(x, y) = tvgutil::ArgUtil::argmax(potentials(x, y));
      }
    }
    return result;
  }
};

}

#endif
