/**
 * grove: PoseCandidate.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_POSECANDIDATE
#define H_GROVE_POSECANDIDATE

#include <boost/shared_ptr.hpp>

#include <ITMLib/Utils/ITMMath.h>

#include <ORUtils/Matrix.h>
#include <ORUtils/MemoryBlock.h>
#include <ORUtils/Vector.h>

namespace grove {

/**
 * \brief This struct represents a candidate camera pose.
 */
struct PoseCandidate
{
  /** The minimum number of points required to estimate a camera pose via the Kabsch algorithm. */
  enum { KABSCH_POINTS = 3 };

  /** The candidate camera pose. */
  Matrix4f cameraPose;

  /** The energy associated to the pose candidate. */
  float energy;

  /** Points in camera reference frame used to estimate the cameraPose. */
  Vector3f pointsCamera[KABSCH_POINTS];

  /** Points in world reference frame used to estimate the cameraPose. */
  Vector3f pointsWorld[KABSCH_POINTS];
};

//#################### OPERATORS ####################

/**
 * \brief Custom operator used to compare candidate camera poses by energy.
 *
 * \param first  A pose candidate.
 * \param second A second pose candidate.
 *
 * \return Whether the energy of the first is lower than the energy of the second candidate.
 */
_CPU_AND_GPU_CODE_
inline bool operator <(const PoseCandidate &first, const PoseCandidate &second)
{
  return first.energy < second.energy;
}

//#################### TYPEDEFS ####################

typedef ORUtils::MemoryBlock<PoseCandidate> PoseCandidateMemoryBlock;
typedef boost::shared_ptr<PoseCandidateMemoryBlock> PoseCandidateMemoryBlock_Ptr;
typedef boost::shared_ptr<const PoseCandidateMemoryBlock> PoseCandidateMemoryBlock_CPtr;

}

#endif
