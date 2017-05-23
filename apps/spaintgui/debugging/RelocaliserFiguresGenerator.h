/**
 * spaintgui: RelocaliserFiguresGenerator.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_SPAINTGUI_RELOCALISERFIGURESGENERATOR
#define H_SPAINTGUI_RELOCALISERFIGURESGENERATOR

#include "../core/Model.h"

namespace spaintgui {

struct RelocaliserFiguresGenerator
{
  static void show_leaf_modes(const Model_Ptr &model);

  static void show_ransac_correspondences(const Model_Ptr &model);

};

}

#endif
