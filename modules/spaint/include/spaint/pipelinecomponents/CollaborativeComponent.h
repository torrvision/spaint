/**
 * spaint: CollaborativeComponent.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_SPAINT_COLLABORATIVECOMPONENT
#define H_SPAINT_COLLABORATIVECOMPONENT

#include <boost/atomic.hpp>
#include <boost/thread.hpp>
#include <boost/timer/timer.hpp>

#include <orx/relocalisation/Relocaliser.h>

#include <tvgutil/numbers/RandomNumberGenerator.h>

#include "CollaborativeContext.h"
#include "../collaboration/CollaborationMode.h"
#include "../collaboration/CollaborativeRelocalisation.h"
#include "../visualisation/VisualisationGenerator.h"

namespace spaint {

/**
 * \brief An instance of this pipeline component can be used to determine the relative poses between agents participating in collaborative SLAM.
 */
class CollaborativeComponent
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The best relocalisation candidate, as chosen by the scheduler. This will be the next relocalisation attempted. */
  boost::shared_ptr<CollaborativeRelocalisation> m_bestCandidate;

  /** The timer used to compute the time spent collaborating. */
  boost::optional<boost::timer::cpu_timer> m_collaborationTimer;

  /** Whether or not to attempt to verify relocalisations that have been reported as poor, not just good ones. */
  bool m_considerPoorRelocalisations;

  /** The shared context needed for collaborative SLAM. */
  CollaborativeContext_Ptr m_context;

  /** TODO */
  std::map<std::string, VoxelRenderState_Ptr> m_depthRenderStates;

  /** The current frame index (in practice, the number of times that run_collaborative_pose_estimation has been called). */
  int m_frameIndex;

  /** The mode in which the collaboration reconstruction should run. */
  CollaborationMode m_mode;

  /** The mutex used to synchronise scheduling and relocalisation. */
  boost::mutex m_mutex;

  /** A condition variable used to tell the relocalisation thread when a candidate relocalisation has been scheduled. */
  boost::condition_variable m_readyToRelocalise;

  /** Whether or not the current reconstruction is consistent (i.e. all scenes are connected to the primary one). */
  bool m_reconstructionIsConsistent;

  /** The thread on which relocalisations should be attempted. */
  boost::thread m_relocalisationThread;

  /** The results of every relocalisation that has been attempted. */
  std::deque<CollaborativeRelocalisation> m_results;

  /** TODO */
  std::map<std::string, VoxelRenderState_Ptr> m_rgbRenderStates;

  /** A random number generator. */
  mutable tvgutil::RandomNumberGenerator m_rng;

  /** Whether or not to stop at the first consistent reconstruction. */
  bool m_stopAtFirstConsistentReconstruction;

  /** A flag used to ensure that the relocalisation thread terminates cleanly when the collaborative component is destroyed. */
  boost::atomic<bool> m_stopRelocalisationThread;

  /** Whether or not to compute the time spent collaborating. */
  bool m_timeCollaboration;

  /** The trajectories followed by the cameras that reconstructed each of the different scenes (only poses where tracking succeeded are stored). */
  std::map<std::string,std::deque<ORUtils::SE3Pose> > m_trajectories;

  /** The indices of the frames that have already been tried when attempting to relocalise one scene against another. */
  std::map<std::pair<std::string,std::string>,std::set<int> > m_triedFrameIndices;

  /** A visualisation generator that is specific to this collaborative component. We avoid sharing one with other components for thread-safety reasons. */
  VisualisationGenerator_CPtr m_visualisationGenerator;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a collaborative component.
   *
   * \param context The shared context needed for collaborative SLAM.
   * \param mode    The mode in which the collaborative reconstruction should run.
   */
  CollaborativeComponent(const CollaborativeContext_Ptr& context, CollaborationMode mode);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the collaborative component.
   */
  ~CollaborativeComponent();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Attempts to estimate the poses of the different scenes involved in the collaborative reconstruction.
   */
  void run_collaborative_pose_estimation();

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief TODO
   */
  std::list<CollaborativeRelocalisation> generate_random_candidates(size_t desiredCandidateCount) const;

  /**
   * \brief TODO
   */
  std::list<CollaborativeRelocalisation> generate_sequential_candidate() const;

  /**
   * \brief Returns whether or not the specified candidate relocalisation is verified.
   *
   * \param candidate The candidate relocalisation.
   * \return          true, if the candidate relocalisation is verified, or false otherwise.
   */
  bool is_verified(const CollaborativeRelocalisation& candidate) const;

  /**
   * \brief Outputs the results of the collaborative pose estimation process (for debugging purposes).
   */
  void output_results() const;

  /**
   * \brief Runs the relocalisation thread, repeatedly attempting scheduled relocalisations until the collaborative component is destroyed.
   */
  void run_relocalisation();

  /**
   * \brief Scores all of the specified candidate relocalisations to allow one of them to be chosen for a relocalisation attempt.
   *
   * \param candidates  The candidate relocalisations to score.
   */
  void score_candidates(std::list<CollaborativeRelocalisation>& candidates) const;

  /**
   * \brief Tries to schedule a candidate relocalisation.
   *
   * \note  If an existing relocalisation attempt is in progress, this will early out and do nothing.
   */
  void try_schedule_relocalisation();

  /**
   * \brief Updates the trajectories for each of the different scenes with the poses of the most recent frames (if successfully tracked).
   *
   * \return  true, if at least one of the scenes is still being reconstructed, or false otherwise.
   */
  bool update_trajectories();
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<CollaborativeComponent> CollaborativeComponent_Ptr;

}

#endif
