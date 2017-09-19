/**
 * spaint: CollaborativeComponent.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_SPAINT_COLLABORATIVECOMPONENT
#define H_SPAINT_COLLABORATIVECOMPONENT

#include "CollaborativeContext.h"
#include "../collaboration/SubmapRelocalisation.h"

namespace spaint {

/**
 * \brief An instance of this pipeline component can be used to determine the relative poses between agents participating in collaborative SLAM.
 */
class CollaborativeComponent
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<SubmapRelocalisation> SubmapRelocalisation_Ptr;
  typedef std::pair<SubmapRelocalisation_Ptr,float> Candidate;

  //#################### CONSTANTS ####################
private:
  /** TODO */
  const float m_failurePenaltyDecreasePerFrame;

  /** TODO */
  const int m_maxRelocalisationsNeeded;

  //#################### PRIVATE VARIABLES ####################
private:
  /** TODO */
  std::list<Candidate> m_candidates;

  /** The shared context needed for collaborative SLAM. */
  CollaborativeContext_Ptr m_context;

  /** TODO */
  std::map<std::pair<std::string,std::string>,float> m_failurePenalties;

  /** TODO */
  int m_frameIndex;

  /** TODO */
  std::list<Candidate> m_redundantCandidates;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a collaborative component.
   *
   * \param context The shared context needed for collaborative SLAM.
   */
  explicit CollaborativeComponent(const CollaborativeContext_Ptr& context);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  void run_collaborative_pose_estimation();

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief TODO
   */
  void add_relocalisation_candidates();

  /**
   * \brief TODO
   */
  void score_relocalisation_candidates();

  /**
   * \brief TODO
   */
  void update_failure_penalties();
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<CollaborativeComponent> CollaborativeComponent_Ptr;

}

#endif
