/**
 * spaintgui: RelocaliserFiguresGenerator.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_SPAINTGUI_RELOCALISERFIGURESGENERATOR
#define H_SPAINTGUI_RELOCALISERFIGURESGENERATOR

#include "../core/Model.h"

/**
 * \brief This struct contains some functions that can be used to generate visualisations as in the CVPR relocalisation paper.
 *        The functions have to be set as frame callbacks in the main spaintgui application.
 */
struct RelocaliserFiguresGenerator
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Saves the modes (centroid and covariances) contained in a predetermined set of leaves as they grow.
   *
   * \param model The model containing the relocaliser.
   */
  static void show_growing_leaf_modes(const Model_Ptr &model);

  /**
   * \brief Prints on screen the modes (centroid and covariances) contained in a predetermined set of leaves.
   *
   * \param model The model containing the relocaliser.
   */
  static void show_leaf_modes(const Model_Ptr &model);

  /**
   * \brief this function allows the inspection of the last candidate poses in a RANSAC pass, rendering and saving views from each.
   *
   * \param model The model containing the relocaliser.
   */
  static void show_ransac_correspondences(const Model_Ptr &model);
};

#endif
