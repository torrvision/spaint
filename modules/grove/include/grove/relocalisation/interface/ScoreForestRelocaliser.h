/**
 * grove: ScoreForestRelocaliser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_GROVE_SCOREFORESTRELOCALISER
#define H_GROVE_SCOREFORESTRELOCALISER

#include "ScoreRelocaliser.h"

namespace grove {

/**
 * \brief An instance of a class deriving from this one can be used to relocalise a camera in a 3D scene using the approach described
 *        in "On-the-Fly Adaptation of Regression Forests for Online Camera Relocalisation" (Cavallari et al., 2017).
 */
class ScoreForestRelocaliser : public ScoreRelocaliser
{
  //#################### CONSTANTS ####################
public:
  enum { FOREST_TREE_COUNT = 5 };

  //#################### TYPEDEFS ####################
public:
  typedef ORUtils::VectorX<int, FOREST_TREE_COUNT> LeafIndices;
  typedef ORUtils::Image<LeafIndices> LeafIndicesImage;
  typedef boost::shared_ptr<LeafIndicesImage> LeafIndicesImage_Ptr;
  typedef boost::shared_ptr<const LeafIndicesImage> LeafIndicesImage_CPtr;

  typedef DecisionForest<DescriptorType, FOREST_TREE_COUNT> ScoreForest;
  typedef boost::shared_ptr<ScoreForest> ScoreForest_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The image containing the indices of the forest leaves associated with the keypoint/descriptor pairs. */
  mutable LeafIndicesImage_Ptr m_leafIndicesImage;

  /** An image in which to store a visualisation of the mapping from pixels to forest leaves (for debugging purposes). */
  mutable ORUChar4Image_Ptr m_pixelsToLeavesImage;

  /** The SCoRe forest on which the relocaliser is based. */
  ScoreForest_Ptr m_scoreForest;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a SCoRe forest relocaliser.
   *
   * \param settings          The settings used to configure the relocaliser.
   * \param settingsNamespace The namespace associated with the settings that are specific to the relocaliser.
   * \param deviceType        The device on which the relocaliser should operate.
   *
   * \throws std::runtime_error If the relocaliser cannot be constructed.
   */
  ScoreForestRelocaliser(const tvgutil::SettingsContainer_CPtr& settings, const std::string& settingsNamespace, ORUtils::DeviceType deviceType);

  //#################### PROTECTED ABSTRACT MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Merges the SCoRe predictions (sets of clusters) associated with each keypoint to create a single
   *        SCoRe prediction (a single set of clusters) for each keypoint.
   *
   * \note  Each keypoint/descriptor pair extracted from the input RGB-D image pairs determines a leaf in a tree of the
   *        forest. Each such leaf contains a set of 3D modal clusters, which together constitute a SCoRe prediction.
   *        This function merges the SCoRe predictions associated with the different leaves (from different trees) with
   *        which each keypoint/descriptor pair is associated, thereby yielding a single SCoRe prediction for each pair.
   *
   * \param leafIndices       An image containing the indices of the leaves (in the different trees) associated with each keypoint/descriptor pair.
   * \param outputPredictions An image into which to store the merged SCoRe predictions.
   */
  virtual void merge_predictions_for_keypoints(const LeafIndicesImage_CPtr& leafIndices, ScorePredictionsImage_Ptr& outputPredictions) const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the prediction associated with the specified leaf in the forest.
   *
   * \param treeIdx The index of the tree containing the leaf.
   * \param leafIdx The index of the leaf.
   * \return        The prediction associated with the specified leaf.
   *
   * \throws std::invalid_argument  If treeIdx or leafIdx are greater than the maximum number of trees or leaves, respectively.
   */
  ScorePrediction get_prediction(uint32_t treeIdx, uint32_t leafIdx) const;

  /**
   * \brief Gets the contents of the reservoir associated with the specified leaf in the forest.
   *
   * \param treeIdx The index of the tree containing the leaf.
   * \param leafIdx The index of the leaf.
   * \return        The reservoir associated with the specified leaf.
   *
   * \throws std::invalid_argument  If treeIdx or leafIdx are greater than the maximum number of trees or leaves, respectively.
   */
  std::vector<Keypoint3DColour> get_reservoir_contents(uint32_t treeIdx, uint32_t leafIdx) const;

  /** Override */
  virtual ORUChar4Image_CPtr get_visualisation_image(const std::string& key) const;

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Checks whether or not the specified leaf is valid, and throws if not.
   *
   * \param treeIdx The index of the tree containing the leaf.
   * \param leafIdx The index of the leaf.
   *
   * \throws std::invalid_argument  If treeIdx or leafIdx are greater than the maximum number of trees or leaves, respectively.
   */
  void ensure_valid_leaf(uint32_t treeIdx, uint32_t leafIdx) const;

  /** Override */
  virtual void make_predictions(const ORUChar4Image *colourImage) const;

  /** Override */
  virtual void make_visualisation_images(const ORFloatImage *depthImage, const std::vector<Result>& results) const;

  /** Override */
  virtual void train_sub(const ORUChar4Image *colourImage, const ORFloatImage *depthImage, const Vector4f& depthIntrinsics, const ORUtils::SE3Pose& cameraPose);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Updates the pixels to leaves image (for debugging purposes).
   *
   * \param depthImage  The current depth image.
   */
  void update_pixels_to_leaves_image(const ORFloatImage *depthImage) const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<ScoreForestRelocaliser> ScoreForestRelocaliser_Ptr;
typedef boost::shared_ptr<const ScoreForestRelocaliser> ScoreForestRelocaliser_CPtr;

}

#endif
