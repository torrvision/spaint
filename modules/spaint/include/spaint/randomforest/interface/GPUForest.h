/**
 * spaint: GPUForest.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_GPUFOREST
#define H_SPAINT_GPUFOREST

#include "../../util/ITMImagePtrTypes.h"

#include "../../features/interface/RGBDPatchFeature.h"
#include "GPUClusterer.h"
#include "GPUReservoir.h"
#include "GPUForestTypes.h"

#include "ORUtils/Vector.h"

#ifdef WITH_SCOREFORESTS
#include <Learner.hpp>
#endif

namespace spaint
{

class GPUForest
{
  // Typedefs
public:
  enum {
    RESERVOIR_SIZE = 1000, // Max number samples in a leaf reservoir
  };

  typedef spaint::LeafIndices LeafIndices; // TODO: remove
  typedef ORUtils::Image<LeafIndices> LeafIndicesImage;
  typedef boost::shared_ptr<LeafIndicesImage> LeafIndicesImage_Ptr;
  typedef boost::shared_ptr<const LeafIndicesImage> LeafIndicesImage_CPtr;

public:
  explicit GPUForest(const std::string &fileName);
  virtual ~GPUForest();

  void reset_predictions();
  void evaluate_forest(const RGBDPatchFeatureImage_CPtr &features,
      GPUForestPredictionsImage_Ptr &predictions);
  void add_features_to_forest(const RGBDPatchFeatureImage_CPtr &features);

  void load_structure_from_file(const std::string &fileName);
  void save_structure_to_file(const std::string &fileName) const;

protected:
  std::vector<int> m_nbNodesPerTree;
  std::vector<int> m_nbLeavesPerTree;

  GPUForestImage_Ptr m_forestImage;
  GPUForestPredictionsBlock_Ptr m_predictionsBlock;
  PositionReservoir_Ptr m_leafReservoirs;
  GPUClusterer_Ptr m_gpuClusterer;

  size_t m_maxReservoirsToUpdate;
  size_t m_reservoirUpdateStartIdx;

  virtual void find_leaves(const RGBDPatchFeatureImage_CPtr &features,
      LeafIndicesImage_Ptr &leaf_indices) const = 0;
  virtual void get_predictions(const LeafIndicesImage_Ptr &leaf_indices,
      GPUForestPredictionsImage_Ptr &predictions) const = 0;

private:
  GPUForest();

  LeafIndicesImage_Ptr m_leafImage;

  //#################### SCOREFOREST INTEROP FUNCTIONS ####################
#ifdef WITH_SCOREFORESTS
public:
  explicit GPUForest(const EnsembleLearner &pretrained_forest);

private:
  int convert_node(const Learner *learner, int node_idx, int tree_idx,
      int n_trees, int output_idx, int first_free_idx,
      GPUForestNode *gpu_nodes);
  void convert_predictions();

  std::vector<PredictionGaussianMean> m_leafPredictions;
  boost::shared_ptr<MeanShift3D> m_ms3D;
#endif
};

typedef boost::shared_ptr<GPUForest> GPUForest_Ptr;
typedef boost::shared_ptr<const GPUForest> GPUForest_CPtr;

}

#endif
