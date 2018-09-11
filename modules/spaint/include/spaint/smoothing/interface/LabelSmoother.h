/**
 * spaint: LabelSmoother.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_LABELSMOOTHER
#define H_SPAINT_LABELSMOOTHER

#include <ORUtils/ImageTypes.h>

#include "../../util/SpaintVoxelScene.h"

namespace spaint {

/**
 * \brief An instance of a class deriving from this one can be used to smooth the labelling of voxels in the scene.
 */
class LabelSmoother
{
  //#################### PROTECTED VARIABLES ####################
protected:
  /** The maximum number of labels that can be in use. */
  const size_t m_maxLabelCount;

  /** The maximum squared distance allowed between the positions of neighbouring voxels if smoothing is to occur. */
  const float m_maxSquaredDistanceBetweenVoxels;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a label smoother.
   *
   * \param maxLabelCount                     The maximum number of labels that can be in use.
   * \param maxSquaredDistanceBetweenVoxels   The maximum squared distance allowed between the positions of neighbouring voxels if smoothing is to occur.
   */
  LabelSmoother(size_t maxLabelCount, float maxSquaredDistanceBetweenVoxels);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the label smoother.
   */
  virtual ~LabelSmoother();

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Smooths the labelling of voxels in the scene, filling in the labels of voxels based on their neighbours.
   *
   * \param raycastResult The raycast result.
   * \param scene         The scene.
   */
  virtual void smooth_labels(const ORFloat4Image *raycastResult, SpaintVoxelScene *scene) const = 0;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<const LabelSmoother> LabelSmoother_CPtr;

}

#endif
