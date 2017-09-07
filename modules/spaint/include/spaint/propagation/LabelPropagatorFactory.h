/**
 * spaint: LabelPropagatorFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_LABELPROPAGATORFACTORY
#define H_SPAINT_LABELPROPAGATORFACTORY

#include <climits>

#include <ITMLib/Utils/ITMLibSettings.h>

#include "interface/LabelPropagator.h"

namespace spaint {

/**
 * \brief This struct can be used to construct label propagators.
 */
struct LabelPropagatorFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes a label propagator.
   *
   * \param raycastResultSize                 The size of the raycast result (in pixels).
   * \param deviceType                        The device on which the label propagator should operate.
   * \param maxAngleBetweenNormals            The largest angle allowed between the normals of neighbouring voxels if propagation is to occur.
   * \param maxSquaredDistanceBetweenColours  The maximum squared distance allowed between the colours of neighbouring voxels if propagation is to occur.
   * \param maxSquaredDistanceBetweenVoxels   The maximum squared distance allowed between the positions of neighbouring voxels if propagation is to occur.
   * \return                                  The label propagator.
   */
  static LabelPropagator_CPtr make_label_propagator(size_t raycastResultSize, ITMLib::ITMLibSettings::DeviceType deviceType,
                                                    float maxAngleBetweenNormals = static_cast<float>(2.0f * M_PI / 180.0f),
                                                    float maxSquaredDistanceBetweenColours = 50.0f * 50.0f,
                                                    float maxSquaredDistanceBetweenVoxels = 10.0f * 10.0f);
};

}

#endif
