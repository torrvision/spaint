/**
 * grove: ScoreRelocaliser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_SCORERELOCALISER
#define H_GROVE_SCORERELOCALISER

#include <boost/optional.hpp>
#include <boost/thread.hpp>

#include <orx/relocalisation/Relocaliser.h>

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
 * \brief An instance of a class deriving from this one can be used to relocalise a camera in a 3D scene by regressing the scene coordinates
 *        corresponding to pixels in the input image and using the resulting correspondences to generate camera pose hypotheses.
 *
 * \note  SCoRe is an acronym for "Scene Coordinate Regression", hence the name of the class.
 */
class ScoreRelocaliser : public orx::Relocaliser
{
  //#################### TYPEDEFS ####################
public:
  typedef Keypoint3DColour ExampleType;
  typedef Keypoint3DColourCluster ClusterType;
  typedef RGBDPatchDescriptor DescriptorType;
  typedef ScorePrediction PredictionType;

  typedef ExampleClusterer<ExampleType, ClusterType, PredictionType::Capacity> Clusterer;
  typedef boost::shared_ptr<Clusterer> Clusterer_Ptr;

  typedef ExampleReservoirs<ExampleType> Reservoirs;
  typedef boost::shared_ptr<Reservoirs> Reservoirs_Ptr;

//#################### PRIVATE VARIABLES ####################
private:
  /** The mutex used to synchronise access to the relocaliser in a multithreaded environment. */
  mutable boost::recursive_mutex m_mutex;

  //#################### PROTECTED VARIABLES ####################
protected:
  /** A flag indicating whether or not this relocaliser is "backed" by another one. */
  bool m_backed;

  /** The sigma of the Gaussian used when computing the example densities (used during clustering). */
  float m_clustererSigma;

  /** The maximum distance there can be between two examples that are part of the same cluster (used during clustering). */
  float m_clustererTau;

  /** The image containing the descriptors extracted from the RGB-D image. */
  RGBDPatchDescriptorImage_Ptr m_descriptorsImage;

  /** The device on which the relocaliser should operate. */
  ORUtils::DeviceType m_deviceType;

  /** Whether or not to produce the debug visualisation images when relocalising. */
  bool m_enableDebugging;

  /** The clusterer used to compute 3D modal clusters from the examples stored in the reservoirs. */
  Clusterer_Ptr m_exampleClusterer;

  /** The feature calculator used to extract keypoints and descriptors from the RGB-D image. */
  DA_RGBDPatchFeatureCalculator_Ptr m_featureCalculator;

  /** The index of the next ground truth pose to use (if the ground truth camera trajectory is available). */
  mutable size_t m_groundTruthFrameIndex;

  /** An image in which to store a visualisation of the ground truth mapping from pixels to world-space points (if available). */
  mutable ORUChar4Image_Ptr m_groundTruthPixelsToPointsImage;

  /** The image containing the ground truth SCoRe predictions associated with the keypoint/descriptor pairs (if available). */
  mutable ScorePredictionsImage_Ptr m_groundTruthPredictionsImage;

  /** The ground truth camera trajectory (if available). */
  boost::optional<std::vector<ORUtils::SE3Pose> > m_groundTruthTrajectory;

  /** The image containing the keypoints extracted from the RGB-D image. */
  Keypoint3DColourImage_Ptr m_keypointsImage;

  /** The maximum number of clusters to store in each reservoir (used during clustering). */
  uint32_t m_maxClusterCount;

  /** The maximum number of relocalisations to output for each call to the relocalise function. */
  uint32_t m_maxRelocalisationsToOutput;

  /** The maximum number of reservoirs to subject to clustering for each call to the update function. */
  uint32_t m_maxReservoirsToUpdate;

  /** The maximum x, y and z coordinates visited by the camera during training. */
  float m_maxX, m_maxY, m_maxZ;

  /** The minimum size of cluster to keep (used during clustering). */
  uint32_t m_minClusterSize;

  /** The minimum x, y and z coordinates visited by the camera during training. */
  float m_minX, m_minY, m_minZ;

  /** An image in which to store a visualisation of the mapping from pixels to world-space points (for debugging purposes). */
  mutable ORUChar4Image_Ptr m_pixelsToPointsImage;

  /** The image containing the SCoRe predictions associated with the keypoint/descriptor pairs. */
  mutable ScorePredictionsImage_Ptr m_predictionsImage;

  /** The Preemptive RANSAC instance, used to estimate the 6DOF camera pose from a set of 3D keypoints and their associated SCoRe predictions. */
  PreemptiveRansac_Ptr m_preemptiveRansac;

  /** The state of the relocaliser. Can be replaced at runtime to relocalise (and train) in a different environment. */
  ScoreRelocaliserState_Ptr m_relocaliserState;

  /** The capacity (maximum size) of each example reservoir. */
  uint32_t m_reservoirCapacity;

  /** The total number of example reservoirs used by the relocaliser. */
  uint32_t m_reservoirCount;

  /** The seed for the random number generators used by the example reservoirs. */
  uint32_t m_rngSeed;

  /** The settings used to configure the relocaliser. */
  tvgutil::SettingsContainer_CPtr m_settings;

  /** The namespace associated with the settings that are specific to the relocaliser. */
  std::string m_settingsNamespace;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a SCoRe relocaliser.
   *
   * \param settings          The settings used to configure the relocaliser.
   * \param settingsNamespace The namespace associated with the settings that are specific to the relocaliser.
   * \param deviceType        The device on which the relocaliser should operate.
   *
   * \throws std::runtime_error If the relocaliser cannot be constructed.
   */
  ScoreRelocaliser(const tvgutil::SettingsContainer_CPtr& settings, const std::string& settingsNamespace, ORUtils::DeviceType deviceType);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the relocaliser.
   */
  virtual ~ScoreRelocaliser();

  //#################### PROTECTED ABSTRACT MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Fills the SCoRe predictions image with a set of clusters for each keypoint extracted from the RGB-D image.
   *
   * \param colourImage The colour image.
   */
  virtual void make_predictions(const ORUChar4Image *colourImage) const = 0;

  /**
   * \brief Trains the relocaliser using information from an RGB-D image pair captured from a known pose in the world.
   *
   * \note  This is a hook function - derived classes should override this rather than overriding train() directly, since train() also does some clustering.
   *
   * \param colourImage     The colour image.
   * \param depthImage      The depth image.
   * \param depthIntrinsics The intrinsic parameters of the depth sensor.
   * \param cameraPose      The position of the camera in the world.
   */
  virtual void train_sub(const ORUChar4Image *colourImage, const ORFloatImage *depthImage, const Vector4f& depthIntrinsics, const ORUtils::SE3Pose& cameraPose) = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void finish_training();

  /**
   * \brief Gets all of the candidate poses that survived the initial culling process during the last run of P-RANSAC,
   *        sorted in non-increasing order of the number of P-RANSAC iterations they survived.
   *
   * \pre   This function should only be called after a prior call to relocalise.
   * \note  The first entry of the vector will be the candidate (if any) returned by the last run of P-RANSAC.
   *
   * \param poseCandidates An output array that will be filled with the candidate poses as described.
   */
  void get_best_poses(std::vector<PoseCandidate>& poseCandidates) const;

  /**
   * \brief Gets the image containing the keypoints extracted from the RGB-D image.
   *
   * \return  The image containing the keypoints extracted from the RGB-D image.
   */
  Keypoint3DColourImage_CPtr get_keypoints_image() const;

  /**
   * \brief Gets the image containing the SCoRe predictions associated with the keypoint/descriptor pairs.
   *
   * \return  The image containing the SCoRe predictions associated with the keypoint/descriptor pairs.
   */
  ScorePredictionsImage_CPtr get_predictions_image() const;

  /** Override */
  virtual ORUChar4Image_CPtr get_visualisation_image(const std::string& key) const;

  /** Override */
  virtual void load_from_disk(const std::string& inputFolder);

  /** Override */
  virtual std::vector<Result> relocalise(const ORUChar4Image *colourImage, const ORFloatImage *depthImage, const Vector4f& depthIntrinsics) const;

  /** Override */
  virtual void reset();

  /** Override */
  virtual void save_to_disk(const std::string& outputFolder) const;

  /**
   * \brief Replaces the relocaliser's current state with that of another relocaliser, and marks this relocaliser as being "backed" by that relocaliser.
   *
   * \note  The new state must previously have been initialised with the right variable sizes.
   *
   * \param backingRelocaliser  The backing relocaliser.
   */
  void set_backing_relocaliser(const boost::shared_ptr<ScoreRelocaliser>& backingRelocaliser);

  /**
   * \brief Sets the ground truth camera trajectory.
   *
   * \param groundTruthTrajectory The ground truth camera trajectory.
   */
  void set_ground_truth_trajectory(const std::vector<ORUtils::SE3Pose>& groundTruthTrajectory);

  /** Override */
  virtual void train(const ORUChar4Image *colourImage, const ORFloatImage *depthImage, const Vector4f& depthIntrinsics, const ORUtils::SE3Pose& cameraPose);

  /** Override */
  virtual void update();

  /**
   * \brief Forcibly updates the contents of every cluster in the example reservoirs.
   *
   * \note  This is computationally intensive, and can require a few hundred milliseconds to terminate.
   */
  void update_all_clusters();

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Makes debug visualisation images to help the user better understand what happened during the most recent attempt to relocalise the camera.
   *
   * \param depthImage  The depth image from which we were trying to relocalise.
   * \param results     The results of the most recent attempt to relocalise.
   */
  virtual void make_visualisation_images(const ORFloatImage *depthImage, const std::vector<Result>& results) const;

  /**
   * \brief Sets a SCoRe prediction for each keypoint that contains a single cluster consisting of the ground truth position of the keypoint in world space.
   *
   * \param keypointsImage    The image containing the keypoints extracted from the RGB-D image.
   * \param cameraToWorld     The ground truth transformation from camera space to world space.
   * \param outputPredictions An image into which to store the SCoRe predictions.
   */
  virtual void set_ground_truth_predictions_for_keypoints(const Keypoint3DColourImage_CPtr& keypointsImage, const Matrix4f& cameraToWorld,
                                                          ScorePredictionsImage_Ptr& outputPredictions) const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Computes the number of reservoirs to subject to clustering during a train/update call.
   *
   * \return  The number of reservoirs to subject to clustering during a train/update call.
   */
  uint32_t compute_nb_reservoirs_to_update() const;

  /**
   * \brief Updates one of the pixels to points images (for debugging purposes).
   *
   * \param worldToCamera       The transformation to use from world to camera space.
   * \param predictionsImage    An image containing SCoRe predictions associated with the keypoint/descriptor pairs.
   * \param pixelsToPointsImage An image in which to store a visualisation of a mapping from pixels to world-space points.
   */
  void update_pixels_to_points_image(const ORUtils::SE3Pose& worldToCamera, const ScorePredictionsImage_Ptr& predictionsImage, ORUChar4Image_Ptr& pixelsToPointsImage) const;

  /**
   * \brief Updates the index of the first reservoir to subject to clustering during the next train/update call.
   */
  void update_reservoir_start_idx();
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<ScoreRelocaliser> ScoreRelocaliser_Ptr;
typedef boost::shared_ptr<const ScoreRelocaliser> ScoreRelocaliser_CPtr;

}

#endif
