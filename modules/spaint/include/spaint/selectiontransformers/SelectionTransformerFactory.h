/**
 * spaint: SelectionTransformerFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_SELECTIONTRANSFORMERFACTORY
#define H_SPAINT_SELECTIONTRANSFORMERFACTORY

#include <boost/shared_ptr.hpp>

#include "interface/SelectionTransformer.h"

namespace spaint {

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<SelectionTransformer> SelectionTransformer_Ptr;
typedef boost::shared_ptr<const SelectionTransformer> SelectionTransformer_CPtr;

/**
 * \brief This class provides functions that construct selection transformers.
 */
class SelectionTransformerFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Makes a voxel to cube selection transformer.
   *
   * \param radius      The (Manhattan) radius (in voxels) to select around each initial voxel.
   * \param deviceType  The device on which the transformer should operate.
   * \return            The selection transformer.
   */
  static SelectionTransformer_Ptr make_voxel_to_cube(int radius, ITMLibSettings::DeviceType deviceType);
};

}

#endif
