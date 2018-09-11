/**
 * spaint: LabelSmoother_CPU.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_LABELSMOOTHER_CPU
#define H_SPAINT_LABELSMOOTHER_CPU

#include "../interface/LabelSmoother.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to smooth the labelling of voxels in the scene using the CPU.
 */
class LabelSmoother_CPU : public LabelSmoother
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a CPU-based label smoother.
   *
   * \param maxLabelCount                     The maximum number of labels that can be in use.
   * \param maxSquaredDistanceBetweenVoxels   The maximum squared distance allowed between the positions of neighbouring voxels if smoothing is to occur.
   */
  LabelSmoother_CPU(size_t maxLabelCount, float maxSquaredDistanceBetweenVoxels);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Smooths the labelling of voxels in the scene, filling in the labels of voxels based on their neighbours.
   *
   * \param raycastResult The raycast result.
   * \param scene         The scene.
   */
  virtual void smooth_labels(const ORFloat4Image *raycastResult, SpaintVoxelScene *scene) const;
};

}

#endif
