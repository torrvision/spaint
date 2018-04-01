/**
 * spaint: LabelSmootherFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_LABELSMOOTHERFACTORY
#define H_SPAINT_LABELSMOOTHERFACTORY

#include <climits>

#include <ORUtils/DeviceType.h>

#include "interface/LabelSmoother.h"

namespace spaint {

/**
 * \brief This struct can be used to construct label smoothers.
 */
struct LabelSmootherFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes a label smoother.
   *
   * \param maxLabelCount                     The maximum number of labels that can be in use.
   * \param deviceType                        The device on which the label smoother should operate.
   * \param maxSquaredDistanceBetweenVoxels   The maximum squared distance allowed between the positions of neighbouring voxels if smoothing is to occur.
   * \return                                  The label smoother.
   */
  static LabelSmoother_CPtr make_label_smoother(size_t maxLabelCount, DeviceType deviceType, float maxSquaredDistanceBetweenVoxels = 10.0f * 10.0f);
};

}

#endif
