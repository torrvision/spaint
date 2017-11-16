/**
 * grove: ScoreRelocaliser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_SCORERELOCALISER
#define H_GROVE_SCORERELOCALISER

#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

#include <ITMLib/Engines/LowLevel/Interface/ITMLowLevelEngine.h>
#include <ITMLib/Utils/ITMImageTypes.h>
#include <ITMLib/Utils/ITMLibSettings.h>
#include <ORUtils/Image.h>
#include <ORUtils/SE3Pose.h>

#include <itmx/base/ITMObjectPtrTypes.h>
#include <itmx/relocalisation/Relocaliser.h>

#include <tvgutil/misc/SettingsContainer.h>

#include "../base/ScoreRelocaliserState.h"

#include "../../clustering/interface/ExampleClusterer.h"
#include "../../features/interface/RGBDPatchFeatureCalculator.h"
#include "../../forests/interface/DecisionForest.h"
#include "../../ransac/interface/PreemptiveRansac.h"
#include "../../reservoirs/interface/ExampleReservoirs.h"
#include "../../scoreforests/Keypoint3DColourCluster.h"
#include "../../scoreforests/ScorePrediction.h"

namespace grove {

/**
 * \brief An instance of a class derived from this one allows the relocalisation of camera pose used to acquire RGB-D
 *        image pairs, according to the method described in:
 *
 *        "On-the-Fly Adaptation of Regression Forests for Online Camera Relocalisation" by
 *        Tommaso Cavallari, Stuart Golodetz*, Nicholas A. Lord*,
 *        Julien Valentin, Luigi Di Stefano and Philip H. S. Torr
 */
class ScoreRelocaliser : public itmx::Relocaliser
{
  //#################### CONSTANTS ####################
public:
  enum { FOREST_TREE_COUNT = 5 };

  //#################### TYPEDEFS ####################
public:
  typedef Keypoint3DColour ExampleType;
  typedef Keypoint3DColourCluster ClusterType;
  typedef RGBDPatchDescriptor DescriptorType;
  typedef ScorePrediction PredictionType;

  typedef ExampleClusterer<ExampleType, ClusterType, PredictionType::Capacity> Clusterer;
  typedef boost::shared_ptr<Clusterer> Clusterer_Ptr;

  typedef ORUtils::VectorX<int, FOREST_TREE_COUNT> LeafIndices;
  typedef ORUtils::Image<LeafIndices> LeafIndicesImage;
  typedef boost::shared_ptr<LeafIndicesImage> LeafIndicesImage_Ptr;
  typedef boost::shared_ptr<const LeafIndicesImage> LeafIndicesImage_CPtr;

  typedef ExampleReservoirs<ExampleType> Reservoirs;
  typedef boost::shared_ptr<Reservoirs> Reservoirs_Ptr;

  typedef DecisionForest<DescriptorType, FOREST_TREE_COUNT> ScoreForest;
  typedef boost::shared_ptr<ScoreForest> ScoreForest_Ptr;

//#################### PRIVATE VARIABLES ####################
private:
  /** An image storing the indices of the forest leaves associated to the keypoint/descriptor pairs. */
  mutable LeafIndicesImage_Ptr m_leafIndicesImage;

  /** An image storing the predictions associated to the keypoint/descriptor pairs. */
  mutable ScorePredictionsImage_Ptr m_predictionsImage;

  /** An image that will store the descriptors extracted from an RGB-D image pair. */
  RGBDPatchDescriptorImage_Ptr m_rgbdPatchDescriptorImage;

  /** An image that will store the keypoints extracted from an RGB-D image pair. */
  Keypoint3DColourImage_Ptr m_rgbdPatchKeypointsImage;

  //#################### PROTECTED VARIABLES ####################
protected:
  /** Sigma used to cluster examples in 3D modal clusters(width of the gaussian used to compute the example density). */
  float m_clustererSigma;

  /** Tau used to cluster examples in modal clusters (maximum distance between examples to be in the same cluster). */
  float m_clustererTau;

  /** The clusterer, used to compute 3D modal clusters from the examples stored in the reservoirs. */
  Clusterer_Ptr m_exampleClusterer;

  /** The feature calculator, used to extract keypoints and describe image patches. */
  DA_RGBDPatchFeatureCalculator_Ptr m_featureCalculator;

  /** The path to the pretrained relocalisation forest structure. */
  std::string m_forestFilename;

  /** A low level engine used to perform basic image processing. */
  LowLevelEngine_Ptr m_lowLevelEngine;

  /** The maximum number of cluster for each leaf in the forest. */
  uint32_t m_maxClusterCount;

  /** The minimum size of cluster to be considered valid. */
  uint32_t m_minClusterSize;

  /**
   * The class implementing the Preemptive-RANSAC algorithm, used to estimate a pose given a set of keypoints and
   * associated modal clusters.
   */
  PreemptiveRansac_Ptr m_preemptiveRansac;

  /** The state of the relocaliser. Can be swapped at runtime with another to relocalise (and train) in a different environment. */
  ScoreRelocaliserState_Ptr m_relocaliserState;

  /** The maximum capacity of the reservoir associated to each leaf in the forest. */
  uint32_t m_reservoirCapacity;

  /** The seed for a random number generator. */
  uint32_t m_rngSeed;

  /** The relocalisaton forest. */
  ScoreForest_Ptr m_scoreForest;

  /** The relocalisation settings. */
  tvgutil::SettingsContainer_CPtr m_settings;

  // Update-related data
  /** The maximum number of reservoirs to subject to clustering for each integration/update call. */
  uint32_t m_maxReservoirsToUpdate;

  /** The total number of example reservoirs in the relocaliser. */
  uint32_t m_reservoirsCount;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs an instance of a ScoreRelocaliser, loading a pretrained forest from a file.
   *
   * \param settings       Pointer to an instance of SettingsContainer used to configure the relocaliser.
   * \param forestFilename The path to the pretrained forest file.
   *
   * \throws std::runtime_error if the forest cannot be loaded.
   */
  ScoreRelocaliser(const tvgutil::SettingsContainer_CPtr& settings, const std::string& forestFilename);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys an instance of ScoreRelocaliser.
   */
  virtual ~ScoreRelocaliser();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Returns the best poses estimated by the last run of the P-RANSAC algorithm.
   *
   * \note  Should be called AFTER calling estimate_pose.
   *
   * \param poseCandidates Output array that will be filled with the best poses estimated by the relocaliser.
   *                       Poses are sorted in descending quality order.
   */
  void get_best_poses(std::vector<PoseCandidate>& poseCandidates) const;

  /**
   * \brief Returns a pointer to an image containing the keypoints used to perform the relocalisation.
   *
   * \return A pointer to an image containing the keypoints used to perform the relocalisation.
   */
  Keypoint3DColourImage_CPtr get_keypoints_image() const;

  /**
   * \brief Returns a pointer to an image containing the forest predictions for each pixel in the keypoint image.
   *
   * \return A pointer to an image containing the forest predictions for each pixel in the keypoint image.
   */
  ScorePredictionsImage_CPtr get_predictions_image() const;

  /**
   * \brief Returns a pointer to the relocaliser state.
   *
   * \return A pointer to the relocaliser state.
   */
  ScoreRelocaliserState_CPtr get_relocaliser_state() const;

  /**
   * \brief Returns a pointer to the relocaliser state (non-const variant).
   *
   * \return A pointer to the relocaliser state.
   */
  ScoreRelocaliserState_Ptr get_relocaliser_state();

  /**
   * \brief Sets the relocaliser state.
   *
   * \note Has to be initialised beforehand, with the right variable sizes.
   *
   * \param relocaliserState The relocaliser state.
   */
  void set_relocaliser_state(const ScoreRelocaliserState_Ptr& relocaliserState);

  /**
   * \brief Updates the contents of each cluster.
   *
   * \note This function is meant to be called once to update every leaf cluster.
   *       It's computationally intensive and requires a few hundred milliseconds to terminate.
   */
  void update_all_clusters();

  //#################### PUBLIC VIRTUAL MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Returns a specific prediction from the forest.
   *
   * \param treeIdx The index of the tree containing the prediciton of interest.
   * \param leafIdx The index of the required leaf prediction.
   *
   * \return The ScorePrediction of interest.
   *
   * \throws std::invalid_argument if either treeIdx or leafIdx are greater than the maximum number of trees or leaves.
   */
  virtual ScorePrediction get_raw_prediction(uint32_t treeIdx, uint32_t leafIdx) const = 0;

  virtual std::vector<Keypoint3DColour> get_reservoir_contents(uint32_t treeIdx, uint32_t leafIdx) const = 0;

  /** Override */
  virtual boost::optional<Result> relocalise(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage, const Vector4f& depthIntrinsics) const;

  /** Override */
  virtual void reset();

  /** Override */
  virtual void train(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage, const Vector4f& depthIntrinsics, const ORUtils::SE3Pose& cameraPose);

  /** Override */
  virtual void update();

  //#################### PROTECTED VIRTUAL ABSTRACT MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Each keypoint/descriptor pair extracted from the input RGB-D image pairs determines a leaf in a tree of the
   *        forest. This function merges the 3D modal clusters associated to multiple leaves coming from different trees
   *        in the forest in a single prediction for each keypoint/descriptor pair.
   *
   * \param leafIndices       Indices of the forest leafs predicted from a keypoint/descriptor pair.
   * \param leafPredictions   A memory block containing all the 3D modal clusters associated to the forest.
   * \param outputPredictions An image wherein each element represent the modal clsters associated to the predicted
   *                          leaves.
   */
  virtual void get_predictions_for_leaves(const LeafIndicesImage_CPtr& leafIndices,
                                          const ScorePredictionsMemoryBlock_CPtr& leafPredictions,
                                          ScorePredictionsImage_Ptr& outputPredictions) const = 0;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Computes the number of reservoirs to subject to clustering during the integration/updating steps.
   *
   * \return The number of reservoirs to subject to clustering.
   */
  uint32_t compute_nb_reservoirs_to_update() const;

  /**
   * \brief Update the index of the reservoir to subject to clustering after the integration/updating steps.
   */
  void update_reservoir_start_idx();
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<ScoreRelocaliser> ScoreRelocaliser_Ptr;
typedef boost::shared_ptr<const ScoreRelocaliser> ScoreRelocaliser_CPtr;

}

#endif
