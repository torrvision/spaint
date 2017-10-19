/**
 * spaint: PoseGraphOptimiser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_SPAINT_POSEGRAPHOPTIMISER
#define H_SPAINT_POSEGRAPHOPTIMISER

#include <set>

#include <boost/atomic.hpp>
#include <boost/optional.hpp>
#include <boost/thread.hpp>

#include <ORUtils/SE3Pose.h>

namespace spaint {

/**
 * \brief TODO
 */
class PoseGraphOptimiser
{
  //#################### TYPEDEFS ####################
public:
  typedef std::pair<std::string,std::string> SceneIDPair;
  typedef std::vector<ORUtils::SE3Pose> SE3PoseCluster;

  //#################### PRIVATE VARIABLES ####################
private:
  /** Estimates of the poses of the different scenes in the global coordinate system. */
  std::map<std::string,ORUtils::SE3Pose> m_estimatedGlobalPoses;

  /** The synchronisation mutex. */
  mutable boost::mutex m_mutex;

  /** The pose graph optimisation thread. */
  boost::shared_ptr<boost::thread> m_optimisationThread;

  /**
   * Accumulated samples of the relative transformations between the different scenes. Each sample for (scene i, scene j)
   * expresses an estimate of the transformation from the coordinate system of scene j to that of scene i.
   */
  std::map<SceneIDPair,std::vector<SE3PoseCluster> > m_relativeTransformSamples;

  /** A condition variable used to wait until new samples have been added. */
  mutable boost::condition_variable m_relativeTransformSamplesAdded;

  /** A flag indicating whether or not samples have been added since the last time a pose graph was constructed. */
  bool m_relativeTransformSamplesChanged;

  /** The IDs of all of the scenes for which a sample has been added. */
  std::set<std::string> m_sceneIDs;

  /** Whether or not the pose graph optimiser should terminate. */
  boost::atomic<bool> m_shouldTerminate;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a pose graph optimiser.
   */
  PoseGraphOptimiser();

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the pose graph optimiser.
   */
  ~PoseGraphOptimiser();

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the number of relocalisations needed between a pair of scenes before we can be fairly confident about the relative transformation between them.
   *
   * \return  The number of relocalisations needed between a pair of scenes before we can be fairly confident about the relative transformation between them.
   */
  static int confidence_threshold();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Adds a sample of the transformation from the coordinate system of scene j to that of scene i.
   *
   * \note  We also add the inverse of the sample passed in as a sample of the transformation from the
   *        coordinate system of scene i to that of scene j.
   *
   * \param sceneI  The ID of scene i.
   * \param sceneJ  The ID of scene j.
   * \param sample  A sample of the transformation from the coordinate system of scene j to that of scene i.
   */
  void add_relative_transform_sample(const std::string& sceneI, const std::string& sceneJ, const ORUtils::SE3Pose& sample);

  /**
   * \brief Terminates the pose graph optimiser.
   */
  void terminate();

  /**
   * \brief Attempts to get the estimated global pose (if any) of the specified scene.
   *
   * \param sceneID The scene whose estimated global pose we want to get.
   * \return        The estimated global pose (if any) of the specified scene, or boost::none otherwise.
   */
  boost::optional<ORUtils::SE3Pose> try_get_estimated_global_pose(const std::string& sceneID) const;

  /**
   * \brief Attempts to get the largest cluster of samples of the transformation from the coordinate system of scene j to that of scene i.
   *
   * \param sceneI  The ID of scene i.
   * \param sceneJ  The ID of scene j.
   * \return        The largest cluster of samples of the transformation from the coordinate system of
   *                scene j to that of scene i, if any such samples exist, or boost::none otherwise.
   */
  boost::optional<SE3PoseCluster> try_get_largest_cluster(const std::string& sceneI, const std::string& sceneJ) const;

  /**
   * \brief Attempts to get an estimate of the transformation from the coordinate system of scene j to that of scene i.
   *
   * \param sceneI  The ID of scene i.
   * \param sceneJ  The ID of scene j.
   * \return        An estimate of the transformation from the coordinate system of scene j to that of scene i,
   *                together with the number of samples it is based on, if possible, or boost::none otherwise.
   */
  boost::optional<std::pair<ORUtils::SE3Pose,size_t> > try_get_relative_transform(const std::string& sceneI, const std::string& sceneJ) const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Adds a sample of the transformation from the coordinate system of scene j to that of scene i.
   *
   * \param sceneI  The ID of scene i.
   * \param sceneJ  The ID of scene j.
   * \param sample  A sample of the transformation from the coordinate system of scene j to that of scene i.
   * \return        true, if the cluster to which the sample was added is now a confident one, or false otherwise.
   */
  bool add_relative_transform_sample_sub(const std::string& sceneI, const std::string& sceneJ, const ORUtils::SE3Pose& sample);

  /**
   * \brief Optimises the relative transformations between the different scenes.
   */
  void run_pose_graph_optimisation();

/**
   * \brief Attempts to get the largest cluster of samples of the transformation from the coordinate system of scene j to that of scene i.
   *
   * \param sceneI  The ID of scene i.
   * \param sceneJ  The ID of scene j.
   * \return        The largest cluster of samples of the transformation from the coordinate system of
   *                scene j to that of scene i, if any such samples exist, or boost::none otherwise.
   */
  boost::optional<SE3PoseCluster> try_get_largest_cluster_sub(const std::string& sceneI, const std::string& sceneJ) const;

  /**
   * \brief Attempts to get an estimate of the transformation from the coordinate system of scene j to that of scene i.
   *
   * \param sceneI  The ID of scene i.
   * \param sceneJ  The ID of scene j.
   * \return        An estimate of the transformation from the coordinate system of scene j to that of scene i,
   *                together with the number of samples it is based on, if possible, or boost::none otherwise.
   */
  boost::optional<std::pair<ORUtils::SE3Pose,size_t> > try_get_relative_transform_sub(const std::string& sceneI, const std::string& sceneJ) const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<PoseGraphOptimiser> PoseGraphOptimiser_Ptr;
typedef boost::shared_ptr<const PoseGraphOptimiser> PoseGraphOptimiser_CPtr;

}

#endif
