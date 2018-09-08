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
  /** TODO */
  boost::shared_ptr<CollaborativeRelocalisation> m_bestCandidate;

  /** The timer used to compute the time spent collaborating. */
  boost::optional<boost::timer::cpu_timer> m_collaborationTimer;

  /** The shared context needed for collaborative SLAM. */
  CollaborativeContext_Ptr m_context;

  /** TODO */
  std::map<std::string, VoxelRenderState_Ptr> m_depthRenderStates;

  /** TODO */
  int m_frameIndex;

  /** TODO */
  CollaborationMode m_mode;

  /** TODO */
  boost::mutex m_mutex;

  /** TODO */
  boost::condition_variable m_readyToRelocalise;

  /** Whether or not the current reconstruction is consistent (i.e. all scenes are connected to the primary one). */
  bool m_reconstructionIsConsistent;

  /** TODO */
  boost::thread m_relocalisationThread;

  /** TODO */
  std::deque<CollaborativeRelocalisation> m_results;

  /** TODO */
  std::map<std::string, VoxelRenderState_Ptr> m_rgbRenderStates;

  /** A random number generator. */
  mutable tvgutil::RandomNumberGenerator m_rng;

  /** Whether or not to stop at the first consistent reconstruction. */
  bool m_stopAtFirstConsistentReconstruction;

  /** TODO */
  boost::atomic<bool> m_stopRelocalisationThread;

  /** Whether or not to compute the time spent collaborating. */
  bool m_timeCollaboration;

  /** TODO */
  std::map<std::string,std::deque<ORUtils::SE3Pose> > m_trajectories;

  /** TODO */
  std::map<std::pair<std::string,std::string>,std::set<int> > m_triedFrameIndices;

  /** A visualisation generator that is specific to this collaborative component. We avoid sharing one with other components for thread-safety reasons. */
  VisualisationGenerator_CPtr m_visualisationGenerator;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a collaborative component.
   *
   * \param context The shared context needed for collaborative SLAM.
   * \param mode    TODO
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
   * \brief TODO
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
   * \brief TODO
   */
  bool is_verified(const CollaborativeRelocalisation& candidate) const;

  /**
   * \brief TODO
   */
  void output_results() const;

  /**
   * \brief TODO
   */
  void run_relocalisation();

  /**
   * \brief TODO
   */
  void score_candidates(std::list<CollaborativeRelocalisation>& candidates) const;

  /**
   * \brief TODO
   */
  void try_schedule_relocalisation();

  /**
   * \brief TODO
   */
  bool update_trajectories();
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<CollaborativeComponent> CollaborativeComponent_Ptr;

}

#endif
