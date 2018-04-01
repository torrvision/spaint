/**
 * grove: PoseCandidate.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_POSECANDIDATE
#define H_GROVE_POSECANDIDATE

#include <boost/shared_ptr.hpp>

#include <ORUtils/Math.h>
#include <ORUtils/MemoryBlock.h>

namespace grove {

/**
 * \brief An instance of this struct represents a candidate camera pose.
 */
struct PoseCandidate
{
  //#################### CONSTANTS ####################

  /** The minimum number of point correspondences needed to estimate a camera pose via the Kabsch algorithm. */
  enum { KABSCH_CORRESPONDENCES_NEEDED = 3 };

  //#################### PUBLIC VARIABLES ####################

  /** The candidate camera pose. */
  Matrix4f cameraPose;

  /** The energy associated with the pose candidate. */
  float energy;

  /** The points in the camera's reference frame that were used to estimate the camera pose. */
  Vector3f pointsCamera[KABSCH_CORRESPONDENCES_NEEDED];

  /** The points in the world reference frame that were used to estimate the camera pose. */
  Vector3f pointsWorld[KABSCH_CORRESPONDENCES_NEEDED];
};

//#################### OPERATORS ####################

/**
 * \brief Compares two pose candidates based on their energies.
 *
 * \param lhs     The first pose candidate.
 * \param rhs     The second pose candidate.
 * \return        true, if the energy of the first pose candidate is less than the energy of the second pose candidate, or false otherwise.
 */
_CPU_AND_GPU_CODE_
inline bool operator<(const PoseCandidate& lhs, const PoseCandidate& rhs)
{
  return lhs.energy < rhs.energy;
}

//#################### TYPEDEFS ####################

typedef ORUtils::MemoryBlock<PoseCandidate> PoseCandidateMemoryBlock;
typedef boost::shared_ptr<PoseCandidateMemoryBlock> PoseCandidateMemoryBlock_Ptr;
typedef boost::shared_ptr<const PoseCandidateMemoryBlock> PoseCandidateMemoryBlock_CPtr;

}

#endif
