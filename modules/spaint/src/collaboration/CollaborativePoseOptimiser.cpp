/**
 * spaint: CollaborativePoseOptimiser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "collaboration/CollaborativePoseOptimiser.h"
using namespace ORUtils;

#include <deque>
#include <fstream>

#include <MiniSlamGraphLib/GraphEdgeSE3.h>
#include <MiniSlamGraphLib/GraphNodeSE3.h>
#include <MiniSlamGraphLib/LevenbergMarquardtMethod.h>
#include <MiniSlamGraphLib/PoseGraph.h>
#include <MiniSlamGraphLib/SlamGraphErrorFunction.h>
using namespace MiniSlamGraph;

#ifdef WITH_GRAPHVIZ
#include <itmx/graphviz/GraphVisualiser.h>
#endif

#ifdef WITH_OPENCV
#include <itmx/ocv/OpenCVUtil.h>
using namespace itmx;
#endif

#include <orx/geometry/GeometryUtil.h>
using namespace orx;

#include <tvgutil/filesystem/PathFinder.h>
#include <tvgutil/timing/TimeUtil.h>
using namespace tvgutil;

namespace bf = boost::filesystem;

#define DEBUGGING 0

namespace spaint {

//#################### CONSTRUCTORS ####################

CollaborativePoseOptimiser::CollaborativePoseOptimiser(const std::string& primarySceneID)
: m_primarySceneID(primarySceneID),
  m_relativeTransformSamplesChanged(false),
  m_shouldTerminate(false)
{}

//#################### DESTRUCTOR ####################

CollaborativePoseOptimiser::~CollaborativePoseOptimiser()
{
  terminate();
}

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

int CollaborativePoseOptimiser::confidence_threshold()
{
  return 2;
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void CollaborativePoseOptimiser::add_relative_transform_sample(const std::string& sceneI, const std::string& sceneJ, const SE3Pose& sample, CollaborationMode mode)
{
  boost::lock_guard<boost::mutex> lock(m_mutex);

  bool signalOptimiser = true;

  if(!add_relative_transform_sample_sub(sceneI, sceneJ, sample, mode)) signalOptimiser = false;
  if(!add_relative_transform_sample_sub(sceneJ, sceneI, SE3Pose(sample.GetInvM()), mode)) signalOptimiser = false;

  m_sceneIDs.insert(sceneI);
  m_sceneIDs.insert(sceneJ);

  if(signalOptimiser)
  {
    m_relativeTransformSamplesChanged = true;
    m_relativeTransformSamplesAdded.notify_one();
  }

#if 1
  std::map<SceneIDPair,std::vector<SE3PoseCluster> >::const_iterator it = m_relativeTransformSamples.find(std::make_pair(sceneI, sceneJ));
  std::cout << "Cluster sizes: ";
  for(size_t i = 0, size = it->second.size(); i < size; ++i)
  {
    std::cout << it->second[i].size() << ' ';
  }
  std::cout << std::endl;
#endif
}

void CollaborativePoseOptimiser::start(const std::string& globalPosesSpecifier)
{
  m_globalPosesSpecifier = globalPosesSpecifier;
  m_optimisationThread.reset(new boost::thread(boost::bind(&CollaborativePoseOptimiser::run_pose_graph_optimisation, this)));
}

void CollaborativePoseOptimiser::terminate()
{
  m_shouldTerminate = true;

  if(m_optimisationThread)
  {
    // Artificially wake up the pose graph optimisation thread and wait for it to terminate.
    m_relativeTransformSamplesAdded.notify_one();
    m_optimisationThread->join();
  }

  save_global_poses();
}

boost::optional<SE3Pose> CollaborativePoseOptimiser::try_get_estimated_global_pose(const std::string& sceneID) const
{
  boost::lock_guard<boost::mutex> lock(m_mutex);
  std::map<std::string,ORUtils::SE3Pose>::const_iterator it = m_estimatedGlobalPoses.find(sceneID);
  return it != m_estimatedGlobalPoses.end() ? boost::optional<SE3Pose>(it->second) : boost::none;
}

boost::optional<CollaborativePoseOptimiser::SE3PoseCluster>
CollaborativePoseOptimiser::try_get_largest_cluster(const std::string& sceneI, const std::string& sceneJ) const
{
  boost::lock_guard<boost::mutex> lock(m_mutex);
  return try_get_largest_cluster_sub(sceneI, sceneJ);
}

boost::optional<std::pair<SE3Pose,size_t> > CollaborativePoseOptimiser::try_get_relative_transform(const std::string& sceneI, const std::string& sceneJ) const
{
  boost::lock_guard<boost::mutex> lock(m_mutex);

  std::map<std::string,ORUtils::SE3Pose>::const_iterator it = m_estimatedGlobalPoses.find(sceneI);
  if(it == m_estimatedGlobalPoses.end()) return try_get_relative_transform_sub(sceneI, sceneJ);

  std::map<std::string,ORUtils::SE3Pose>::const_iterator jt = m_estimatedGlobalPoses.find(sceneJ);
  if(jt == m_estimatedGlobalPoses.end()) return try_get_relative_transform_sub(sceneI, sceneJ);

  return std::make_pair(SE3Pose(it->second.GetM() * jt->second.GetInvM()), static_cast<size_t>(confidence_threshold()));
}

boost::optional<std::vector<CollaborativePoseOptimiser::SE3PoseCluster> >
CollaborativePoseOptimiser::try_get_relative_transform_samples(const std::string& sceneI, const std::string& sceneJ) const
{
  boost::lock_guard<boost::mutex> lock(m_mutex);

  std::map<SceneIDPair,std::vector<SE3PoseCluster> >::const_iterator it = m_relativeTransformSamples.find(std::make_pair(sceneI, sceneJ));
  return it != m_relativeTransformSamples.end() ? boost::optional<std::vector<SE3PoseCluster> >(it->second) : boost::none;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

bool CollaborativePoseOptimiser::add_relative_transform_sample_sub(const std::string& sceneI, const std::string& sceneJ, const SE3Pose& sample, CollaborationMode mode)
{
#if DEBUGGING
  std::cout << "Adding sample: " << sceneI << "<-" << sceneJ << '\n'
            << GeometryUtil::to_matlab(sample.GetM()) << '\n';
#endif

  // Try to find an existing cluster that contains a sample that is sufficiently similar to the new sample.
  // If we find one, add the sample to that cluster and early out.
  std::vector<SE3PoseCluster>& clusters = m_relativeTransformSamples[std::make_pair(sceneI, sceneJ)];
  for(size_t i = 0, clusterCount = clusters.size(); i < clusterCount; ++i)
  {
    for(size_t j = 0, size = clusters[i].size(); j < size; ++j)
    {
      if(GeometryUtil::poses_are_similar(sample, clusters[i][j], 20 * M_PI / 180, 0.1f))
      {
        clusters[i].push_back(sample);

        if(clusters[i].size() >= confidence_threshold())
        {
          if(mode == CM_LIVE)
          {
            // TODO: Hysteresis
            for(size_t k = 0; k < clusterCount; ++k)
            {
              if(k != i && clusters[k].size() > 1)
              {
                clusters[k].pop_back();
              }
            }
          }

          return true;
        }
        else return false;
      }
    }
  }

  // If the new sample is not sufficiently similar to the samples in any of the existing clusters, create a new cluster for it.
  SE3PoseCluster newCluster;
  newCluster.push_back(sample);
  clusters.push_back(newCluster);
  return newCluster.size() >= confidence_threshold();
}

void CollaborativePoseOptimiser::run_pose_graph_optimisation()
{
  std::cout << "Starting pose graph optimisation thread" << std::endl;

  while(!m_shouldTerminate)
  {
    PoseGraph graph;
    std::vector<std::string> sceneIDs;

    {
      boost::unique_lock<boost::mutex> lock(m_mutex);

      // Wait for samples to be added or the termination flag to be set.
      while(!m_relativeTransformSamplesChanged && !m_shouldTerminate) m_relativeTransformSamplesAdded.wait(lock);

      // If the termination flag is set, early out.
      if(m_shouldTerminate) return;

      // Reset the change flag.
      m_relativeTransformSamplesChanged = false;

      // Determine the ID of the primary scene.
      sceneIDs = std::vector<std::string>(m_sceneIDs.begin(), m_sceneIDs.end());
      const int sceneCount = static_cast<int>(sceneIDs.size());
      int primarySceneID = -1;
      for(int i = 0; i < sceneCount; ++i)
      {
        if(sceneIDs[i] == m_primarySceneID)
        {
          primarySceneID = i;
          break;
        }
      }

      // If no sample has been added for the primary scene yet, we can't build a valid pose graph.
      if(primarySceneID == -1) continue;

      // Determine which connections between the scenes are confident ones.
      std::deque<std::deque<bool> > confidentlyConnected(sceneCount);
      for(int i = 0; i < sceneCount; ++i)
      {
        confidentlyConnected[i].resize(sceneCount);
      }

      for(int i = 0; i < sceneCount; ++i)
      {
        confidentlyConnected[i][i] = true;

        for(int j = 0; j < sceneCount; ++j)
        {
          if(j == i) continue;

          boost::optional<std::pair<SE3Pose,size_t> > relativeTransform = try_get_relative_transform_sub(sceneIDs[i], sceneIDs[j]);
          if(relativeTransform && relativeTransform->second >= confidence_threshold())
          {
            confidentlyConnected[i][j] = confidentlyConnected[j][i] = true;
          }
        }
      }

      // Use a variant of Floyd-Warshall to compute the reflexive transitive closure of the confident connections.
      for(int k = 0; k < sceneCount; ++k)
      {
        for(int i = 0; i < sceneCount; ++i)
        {
          for(int j = 0; j < sceneCount; ++j)
          {
            if(confidentlyConnected[i][k] && confidentlyConnected[k][j])
            {
              confidentlyConnected[i][j] = true;
            }
          }
        }
      }

#if DEBUGGING
      // Output the reflexive transitive closure of the confident connections for debugging purposes.
      for(int i = 0; i < sceneCount; ++i)
      {
        for(int j = 0; j < sceneCount; ++j)
        {
          std::cout << confidentlyConnected[i][j] << ' ';
        }
        std::cout << '\n';
      }
#endif

      // Add any edge with at least one endpoint that is confidently connected to the primary scene to the pose graph.
      bool graphHasEdges = false;
      std::string edgeDesc;
      for(int i = 0; i < sceneCount; ++i)
      {
        for(int j = 0; j < sceneCount; ++j)
        {
          if(j == i) continue;

          edgeDesc += sceneIDs[j] + " -> " + sceneIDs[i];

          if(!confidentlyConnected[i][primarySceneID] && !confidentlyConnected[j][primarySceneID])
          {
            edgeDesc += ";\n";
            continue;
          }

          boost::optional<std::pair<SE3Pose,size_t> > relativeTransform = try_get_relative_transform_sub(sceneIDs[i], sceneIDs[j]);
          if(!relativeTransform || relativeTransform->second < confidence_threshold())
          {
            edgeDesc += ";\n";
            continue;
          }

#if DEBUGGING
          std::cout << "Relative Transform (" << i << '/' << sceneIDs[i] << " <- " << j << '/' << sceneIDs[j] << "): " << relativeTransform->second << '\n'
                    << GeometryUtil::to_matlab(relativeTransform->first.GetM()) << '\n';
#endif

          GraphEdgeSE3 *edge = new GraphEdgeSE3;
          edge->setFromNodeId(j);
          edge->setToNodeId(i);
          edge->setMeasurementSE3(relativeTransform->first);
          graph.addEdge(edge);

          edgeDesc += " [color=red];\n";

          graphHasEdges = true;
        }
      }

      // If no scenes are currently confidently connected to the primary scene, we can't build a valid pose graph.
      if(!graphHasEdges) continue;

      // Add a node for each scene that is confidently connected to the primary scene to the pose graph.
      std::string nodeDesc;
      for(int i = 0; i < sceneCount; ++i)
      {
        nodeDesc += sceneIDs[i];

        if(!confidentlyConnected[i][primarySceneID])
        {
          nodeDesc += ";\n";
          continue;
        }

        /*std::map<std::string,ORUtils::SE3Pose>::const_iterator jt = m_estimatedGlobalPoses.find(sceneIDs[i]);*/

        GraphNodeSE3 *node = new GraphNodeSE3;
        node->setId(i);
        node->setPose(/*jt != m_estimatedGlobalPoses.end() ? jt->second : */ORUtils::SE3Pose());
        node->setFixed(i == primarySceneID);
        graph.addNode(node);

        nodeDesc += " [fillcolor=cyan];\n";
      }

#if 1
      std::cout << "Finished creating pose graph for optimisation" << std::endl;
#endif

#if defined(WITH_GRAPHVIZ) && defined(WITH_OPENCV) && DEBUGGING
      static GraphVisualiser gv;
      ORUChar4Image_Ptr img = gv.generate_visualisation("digraph { node [ shape=rectangle, style=filled, fillcolor=white];\n" + nodeDesc + edgeDesc + " }");
      cv::Mat3b cvImg = OpenCVUtil::make_rgb_image(img->GetData(MEMORYDEVICE_CPU), img->noDims.x, img->noDims.y);
      cv::imshow("Pose Graph", cvImg);
      cv::waitKey(1);
#endif
    }

    // Run the pose graph optimisation.
    graph.prepareEvaluations();
    SlamGraphErrorFunction errFunc(graph);
    SlamGraphErrorFunction::Parameters params(graph);
    LevenbergMarquardtMethod::minimize(errFunc, params);
    graph.setNodeIndex(params.getNodes());

    {
      boost::lock_guard<boost::mutex> lock(m_mutex);

      // Extract and store the optimised poses.
      for(int i = 0, sceneCount = static_cast<int>(sceneIDs.size()); i < sceneCount; ++i)
      {
        SlamGraph::NodeIndex::const_iterator jt = graph.getNodeIndex().find(i);
        if(jt == graph.getNodeIndex().end()) continue;

        const GraphNodeSE3 *node = static_cast<const GraphNodeSE3*>(jt->second);
        m_estimatedGlobalPoses[sceneIDs[i]] = node->getPose();

#if DEBUGGING
        std::cout << "Estimated Pose (" << i << '/' << sceneIDs[i] << "): " << GeometryUtil::to_matlab(node->getPose().GetM()) << '\n';
#endif
      }
    }
  }
}

void CollaborativePoseOptimiser::save_global_poses() const
{
  // If there aren't any poses to save, early out.
  if(m_estimatedGlobalPoses.empty()) return;

  // Determine the file to which to save the poses.
  const std::string dirName = "global_poses";
  const std::string globalPosesSpecifier = m_globalPosesSpecifier != "" ? m_globalPosesSpecifier : TimeUtil::get_iso_timestamp();
  const bf::path p = find_subdir_from_executable(dirName) / (globalPosesSpecifier + ".txt");

  // Try to ensure that the directory into which we want to save the file exists. If we can't, early out.
  try
  {
    bf::create_directories(p.parent_path());
  }
  catch(bf::filesystem_error&)
  {
    std::cerr << "Warning: Could not create directory '" << dirName << "'\n";
    return;
  }

  // Try to save the poses into the file. If we can't, early out.
  std::ofstream fs(p.string().c_str());
  if(!fs)
  {
    std::cerr << "Warning: Could not open file to save global poses\n";
    return;
  }

  for(std::map<std::string,SE3Pose>::const_iterator it = m_estimatedGlobalPoses.begin(), iend = m_estimatedGlobalPoses.end(); it != iend; ++it)
  {
    DualQuatd dq = GeometryUtil::pose_to_dual_quat<double>(it->second);
    fs << it->first << ' ' << dq << '\n';
  }
}

boost::optional<CollaborativePoseOptimiser::SE3PoseCluster>
CollaborativePoseOptimiser::try_get_largest_cluster_sub(const std::string& sceneI, const std::string& sceneJ) const
{
  // Try to look up the sample clusters of the relative transformation from the coordinate system of scene j to that of scene i.
  std::map<SceneIDPair,std::vector<SE3PoseCluster> >::const_iterator it = m_relativeTransformSamples.find(std::make_pair(sceneI, sceneJ));

  // If there aren't any, it's because we haven't found the relative transformation between the two scenes yet, so early out.
  if(it == m_relativeTransformSamples.end()) return boost::none;

  // Otherwise, find a largest cluster and return it.
  const std::vector<SE3PoseCluster>& clusters = it->second;
  const SE3PoseCluster *largestCluster = NULL;
  size_t largestClusterSize = 0;
  for(size_t i = 0, clusterCount = clusters.size(); i < clusterCount; ++i)
  {
    size_t clusterSize = clusters[i].size();
    if(clusterSize > largestClusterSize)
    {
      largestCluster = &clusters[i];
      largestClusterSize = clusterSize;
    }
  }

  return largestCluster ? boost::optional<SE3PoseCluster>(*largestCluster) : boost::none;
}

boost::optional<std::pair<SE3Pose,size_t> > CollaborativePoseOptimiser::try_get_relative_transform_sub(const std::string& sceneI, const std::string& sceneJ) const
{
  boost::optional<SE3PoseCluster> largestCluster = try_get_largest_cluster_sub(sceneI, sceneJ);
  return largestCluster ? boost::optional<std::pair<SE3Pose,size_t> >(std::make_pair(GeometryUtil::blend_poses(*largestCluster), largestCluster->size())) : boost::none;
}

}
