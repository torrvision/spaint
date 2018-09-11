/**
 * spaint: SemanticVisualiser_CPU.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_SEMANTICVISUALISER_CPU
#define H_SPAINT_SEMANTICVISUALISER_CPU

#include "../interface/SemanticVisualiser.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to render a semantic visualisation of an InfiniTAM scene using the CPU.
 */
class SemanticVisualiser_CPU : public SemanticVisualiser
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a CPU-based semantic visualiser.
   *
   * \param maxLabelCount The maximum number of labels that can be in use.
   */
  explicit SemanticVisualiser_CPU(size_t maxLabelCount);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /** Override */
  virtual void render_internal(const SpaintVoxelScene *scene, const ORUtils::SE3Pose *pose, const ITMLib::ITMIntrinsics *intrinsics, const ITMLib::ITMRenderState *renderState,
                               LightingType lightingType, float labelAlpha, ORUChar4Image *outputImage) const;
};

}

#endif
