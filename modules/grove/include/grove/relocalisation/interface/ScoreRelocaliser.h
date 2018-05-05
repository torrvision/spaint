/**
 * grove: ScoreRelocaliser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_SCORERELOCALISER
#define H_GROVE_SCORERELOCALISER

#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

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
 * \brief An instance of a class deriving from this one can be used to relocalise a camera in a 3D scene, using the approach described
 *        in "On-the-Fly Adaptation of Regression Forests for Online Camera Relocalisation" (Cavallari et al., 2017).
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
  /** An image that will store the indices of the forest leaves associated with the keypoint/descriptor pairs. */
  mutable LeafIndicesImage_Ptr m_leafIndicesImage;

  /** An image that will store the predictions associated with the keypoint/descriptor pairs. */
  mutable ScorePredictionsImage_Ptr m_predictionsImage;

  /** An image that will store the descriptors extracted from an RGB-D image. */
  RGBDPatchDescriptorImage_Ptr m_rgbdPatchDescriptorImage;

  /** An image that will store the keypoints extracted from an RGB-D image. */
  Keypoint3DColourImage_Ptr m_rgbdPatchKeypointsImage;

  //#################### PROTECTED VARIABLES ####################
protected:
  /** The sigma of the Gaussian used when computing the example densities (used during clustering). */
  float m_clustererSigma;

  /** The maximum distance there can be between two examples that are part of the same cluster (used during clustering). */
  float m_clustererTau;

  /** The device on which the relocaliser should operate. */
  DeviceType m_deviceType;

  /** The clusterer used to compute 3D modal clusters from the examples stored in the reservoirs. */
  Clusterer_Ptr m_exampleClusterer;

  /** The feature calculator used to extract keypoints and descriptors from an RGB-D image. */
  DA_RGBDPatchFeatureCalculator_Ptr m_featureCalculator;

  /** The low-level engine used to perform basic image processing. */
  LowLevelEngine_Ptr m_lowLevelEngine;

  /** The maximum number of clusters to store in each leaf in the forest (used during clustering). */
  uint32_t m_maxClusterCount;

  /** The maximum number of relocalisations to output for each call to the relocalise function. */
  uint32_t m_maxRelocalisationsToOutput;

  /** The maximum number of reservoirs to subject to clustering for each call to the update function. */
  uint32_t m_maxReservoirsToUpdate;

  /** The minimum size of cluster to keep (used during clustering). */
  uint32_t m_minClusterSize;

  /**
   * The class implementing the Preemptive-RANSAC algorithm, used to estimate a pose given a set of keypoints and
   * associated modal clusters.
   */
  PreemptiveRansac_Ptr m_preemptiveRansac;

  /** The state of the relocaliser. Can be swapped at runtime with another to relocalise (and train) in a different environment. */
  ScoreRelocaliserState_Ptr m_relocaliserState;

  /** The capacity (maximum size) of each reservoir associated with a leaf in the forest. */
  uint32_t m_reservoirCapacity;

  /** The total number of example reservoirs used by the relocaliser (in practice, this is equal to the number of leaves in the forest). */
  uint32_t m_reservoirCount;

  /** The seed for the random number generators used by the example reservoirs. */
  uint32_t m_rngSeed;

  /** The SCoRe forest on which the relocaliser is based. */
  ScoreForest_Ptr m_scoreForest;

  /** The settings used to configure the relocaliser. */
  tvgutil::SettingsContainer_CPtr m_settings;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a SCoRe relocaliser by loading a pre-trained forest from a file.
   *
   * \param forestFilename  The name of the file from which to load the pre-trained forest.
   * \param settings        The settings used to configure the relocaliser.
   * \param deviceType      The device on which the relocaliser should operate.
   *
   * \throws std::runtime_error If the forest cannot be loaded.
   */
  ScoreRelocaliser(const std::string& forestFilename, const tvgutil::SettingsContainer_CPtr& settings, DeviceType deviceType);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the relocaliser.
   */
  virtual ~ScoreRelocaliser();

  //#################### PROTECTED ABSTRACT MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Each keypoint/descriptor pair extracted from the input RGB-D image pairs determines a leaf in a tree of the
   *        forest. This function merges the 3D modal clusters associated to multiple leaves coming from different trees
   *        in the forest in a single prediction for each keypoint/descriptor pair.
   *
   * \param leafIndices       Indices of the forest leafs predicted from a keypoint/descriptor pair.
   * \param leafPredictions   A memory block containing all the 3D modal clusters associated to the forest.
   * \param outputPredictions An image wherein each element represent the modal clusters associated to the predicted leaves.
   */
  virtual void get_predictions_for_leaves(const LeafIndicesImage_CPtr& leafIndices, const ScorePredictionsMemoryBlock_CPtr& leafPredictions,
                                          ScorePredictionsImage_Ptr& outputPredictions) const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void finish_training();

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
   * \brief Returns a specific prediction from the forest.
   *
   * \param treeIdx The index of the tree containing the prediction of interest.
   * \param leafIdx The index of the required leaf prediction.
   *
   * \return The ScorePrediction of interest.
   *
   * \throws std::invalid_argument if either treeIdx or leafIdx are greater than the maximum number of trees or leaves.
   */
  ScorePrediction get_raw_prediction(uint32_t treeIdx, uint32_t leafIdx) const;

  /**
   * \brief Returns a pointer to the relocaliser state (non-const variant).
   *
   * \return A pointer to the relocaliser state.
   */
  ScoreRelocaliserState_Ptr get_relocaliser_state();

  /**
   * \brief Returns a pointer to the relocaliser state.
   *
   * \return A pointer to the relocaliser state.
   */
  ScoreRelocaliserState_CPtr get_relocaliser_state() const;

  /**
   * \brief TODO
   */
  std::vector<Keypoint3DColour> get_reservoir_contents(uint32_t treeIdx, uint32_t leafIdx) const;

  /** Override */
  virtual void load_from_disk(const std::string& inputFolder);

  /** Override */
  virtual std::vector<Result> relocalise(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage, const Vector4f& depthIntrinsics) const;

  /** Override */
  virtual void reset();

  /** Override */
  virtual void save_to_disk(const std::string& outputFolder) const;

  /**
   * \brief Sets the relocaliser state.
   *
   * \note Has to be initialised beforehand, with the right variable sizes.
   *
   * \param relocaliserState The relocaliser state.
   */
  void set_relocaliser_state(const ScoreRelocaliserState_Ptr& relocaliserState);

  /** Override */
  virtual void train(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage, const Vector4f& depthIntrinsics, const ORUtils::SE3Pose& cameraPose);

  /** Override */
  virtual void update();

  /**
   * \brief Updates the contents of each cluster.
   *
   * \note This function is meant to be called once to update every leaf cluster.
   *       It's computationally intensive and requires a few hundred milliseconds to terminate.
   */
  void update_all_clusters();

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
