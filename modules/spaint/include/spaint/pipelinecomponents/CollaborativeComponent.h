/**
 * spaint: CollaborativeComponent.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_SPAINT_COLLABORATIVECOMPONENT
#define H_SPAINT_COLLABORATIVECOMPONENT

#include <boost/atomic.hpp>
#include <boost/thread.hpp>

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

  //#################### NESTED TYPES ####################
private:
  /**
   * \brief TODO
   */
  struct ScenePairInfo
  {
    /** TODO */
    std::list<Candidate> m_candidates;

    /** TODO */
    std::vector<ORUtils::SE3Pose> m_triedLocalPoses;
  };

  //#################### PRIVATE VARIABLES ####################
private:
  /** TODO */
  SubmapRelocalisation_Ptr m_bestCandidate;

  /** The shared context needed for collaborative SLAM. */
  CollaborativeContext_Ptr m_context;

  /** TODO */
  int m_frameIndex;

  /** TODO */
  boost::mutex m_mutex;

  /** TODO */
  size_t m_nextScenePairIndex;

  /** TODO */
  boost::condition_variable m_readyToRelocalise;

  /** TODO */
  boost::thread m_relocalisationThread;

  /** TODO */
  std::map<std::pair<std::string,std::string>,ScenePairInfo> m_scenePairInfos;

  /** TODO */
  boost::atomic<bool> m_stopRelocalisationThread;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a collaborative component.
   *
   * \param context The shared context needed for collaborative SLAM.
   */
  explicit CollaborativeComponent(const CollaborativeContext_Ptr& context);

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
  void add_relocalisation_candidates();

  /**
   * \brief TODO
   */
  void run_relocalisation();

  /**
   * \brief TODO
   */
  void try_schedule_relocalisation();
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<CollaborativeComponent> CollaborativeComponent_Ptr;

}

#endif
