/**
 * spaint: SCoReForest.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SCOREFOREST
#define H_SPAINT_SCOREFOREST

#include "../../util/ITMImagePtrTypes.h"

#include "../../features/interface/RGBDPatchFeature.h"
#include "GPUClusterer.h"
#include "GPUReservoir.h"
#include "../SCoReForestTypes.h"

#include "ORUtils/Vector.h"

#ifdef WITH_SCOREFORESTS
#include <Learner.hpp>
#endif

namespace spaint
{

class SCoReForest
{
  // Typedefs
public:
  struct NodeEntry
  {
    int leftChildIdx; // No need to store the right child, it's left + 1
    int leafIdx;    // Index of the associated leaf (-1 if the node is not a leaf)
    int featureIdx;   // Index of the feature to evaluate;
    float featureThreshold; // Feature threshold
  };

  typedef ORUtils::Image<NodeEntry> NodeImage;
  typedef boost::shared_ptr<ORUtils::Image<NodeEntry> > NodeImage_Ptr;
  typedef boost::shared_ptr<const ORUtils::Image<NodeEntry> > NodeImage_CPtr;

  enum
  {
    RESERVOIR_SIZE = 1024, // Max number samples in a leaf reservoir
  };

  typedef spaint::LeafIndices LeafIndices; // TODO: remove
  typedef ORUtils::Image<LeafIndices> LeafIndicesImage;
  typedef boost::shared_ptr<LeafIndicesImage> LeafIndicesImage_Ptr;
  typedef boost::shared_ptr<const LeafIndicesImage> LeafIndicesImage_CPtr;

public:
  explicit SCoReForest(const std::string &fileName);
  virtual ~SCoReForest();

  void reset_predictions();
  void evaluate_forest(const RGBDPatchFeatureImage_CPtr &features,
      GPUForestPredictionsImage_Ptr &predictions);
  void add_features_to_forest(const RGBDPatchFeatureImage_CPtr &features);
  void update_forest();

  void load_structure_from_file(const std::string &fileName);
  void save_structure_to_file(const std::string &fileName) const;

  size_t get_nb_trees() const;
  size_t get_nb_nodes_in_tree(size_t treeIdx) const;
  size_t get_nb_leaves_in_tree(size_t treeIdx) const;
  virtual GPUForestPrediction get_prediction(size_t treeIdx,
      size_t leafIdx) const = 0;

protected:
  std::vector<int> m_nbNodesPerTree;
  std::vector<int> m_nbLeavesPerTree;

  NodeImage_Ptr m_nodeImage;
  GPUForestPredictionsBlock_Ptr m_predictionsBlock;
  PositionReservoir_Ptr m_leafReservoirs;
  GPUClusterer_Ptr m_gpuClusterer;

  size_t m_maxReservoirsToUpdate;
  size_t m_lastFeaturesAddedStartIdx;
  size_t m_reservoirUpdateStartIdx;

  virtual void find_leaves(const RGBDPatchFeatureImage_CPtr &features,
      LeafIndicesImage_Ptr &leaf_indices) const = 0;
  virtual void get_predictions(const LeafIndicesImage_Ptr &leaf_indices,
      GPUForestPredictionsImage_Ptr &predictions) const = 0;

private:
  SCoReForest();

  LeafIndicesImage_Ptr m_leafImage;

  //#################### SCOREFOREST INTEROP FUNCTIONS ####################
#ifdef WITH_SCOREFORESTS
public:
  explicit SCoReForest(const EnsembleLearner &pretrained_forest);

private:
  int convert_node(const Learner *learner, int node_idx, int tree_idx,
      int n_trees, int output_idx, int first_free_idx,
      NodeEntry *gpu_nodes);
  void convert_predictions();

  std::vector<PredictionGaussianMean> m_leafPredictions;
  boost::shared_ptr<MeanShift3D> m_ms3D;
#endif
};

typedef boost::shared_ptr<SCoReForest> SCoReForest_Ptr;
typedef boost::shared_ptr<const SCoReForest> SCoReForest_CPtr;

}

#endif
