/**
 * grove: ScoreNetRelocaliser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_GROVE_SCORENETRELOCALISER
#define H_GROVE_SCORENETRELOCALISER

#ifdef _MSC_VER
  // Undefine NOMINMAX if it's defined, since otherwise we'll get a warning when Torch defines it.
  #ifdef NOMINMAX
    #undef NOMINMAX
  #endif

  // Suppress some VC++ warnings that are produced when including the Torch headers.
  #pragma warning(disable:4018 4068 4146 4244 4251 4267 4275 4297 4522 4800)
#endif

#include <torch/script.h>

#ifdef _MSC_VER
  // Reenable the suppressed warnings for the rest of the translation unit.
  #pragma warning(default:4018 4068 4146 4244 4251 4267 4275 4297 4522 4800)
#endif

#include <tvgutil/numbers/RandomNumberGenerator.h>

#include "ScoreRelocaliser.h"

namespace grove {

/**
 * \brief An instance of a class deriving from this one can be used to relocalise a camera in a 3D scene by adapting the predictions of a
 *        pre-trained scene coordinate regression network to the scene of interest.
 */
class ScoreNetRelocaliser : public ScoreRelocaliser
{
  //#################### TYPEDEFS ####################
protected:
  typedef ORUtils::VectorX<int, 1> BucketIndices;
  typedef ORUtils::Image<BucketIndices> BucketIndicesImage;
  typedef boost::shared_ptr<BucketIndicesImage> BucketIndicesImage_Ptr;
  typedef boost::shared_ptr<const BucketIndicesImage> BucketIndicesImage_CPtr;

  typedef boost::shared_ptr<ORUtils::MemoryBlock<float> > ScoreNetOutput_Ptr;
  typedef boost::shared_ptr<const ORUtils::MemoryBlock<float> > ScoreNetOutput_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** An image containing the bucket (example reservoir) indices associated with the keypoints. */
  BucketIndicesImage_Ptr m_bucketIndicesImage;

  /** A mapping from indices of cells in the grid placed over the training scene to example reservoir indices. */
  // FIXME: This should be in the relocaliser state, not in the relocaliser itself.
  mutable std::map<int,int> m_bucketRemapper;

  /** The size of each bucket (in cm). */
  int m_bucketSizeCm;

  /** The type of network being used (dsac|vgg). */
  std::string m_netType;

  /** Whether or not to reuse a randomly chosen reservoir when no more are available, as opposed to deterministically choosing one to reuse. */
  bool m_reuseRandomWhenFull;

  /** A random number generator. */
  tvgutil::RandomNumberGenerator_Ptr m_rng;

  /** The size assumed for the overall scene (in cm). In practice, this will usually be an upper bound on the maximum scene size expected. */
  int m_sceneSizeCm;

  /** The SCoRe network on which the relocaliser is based. */
  std::shared_ptr<torch::jit::script::Module> m_scoreNet;

  /** A memory block into which to copy the output tensor produced by the SCoRe network for downstream processing. */
  ScoreNetOutput_Ptr m_scoreNetOutput;

  /** Whether or not to use the bucket predictions in preference to the raw output of the network (necessary if testing on a scene other than the training scene). */
  bool m_useBucketPredictions;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a network-based SCoRe relocaliser.
   *
   * \param settings          The settings used to configure the relocaliser.
   * \param settingsNamespace The namespace associated with the settings that are specific to the relocaliser.
   * \param deviceType        The device on which the relocaliser should operate.
   *
   * \throws std::runtime_error If the relocaliser cannot be constructed.
   */
  ScoreNetRelocaliser(const tvgutil::SettingsContainer_CPtr& settings, const std::string& settingsNamespace, ORUtils::DeviceType deviceType);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void reset();

  //#################### PROTECTED ABSTRACT MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Sets a SCoRe prediction for each keypoint that contains all of the clusters in the bucket (example reservoir)
   *        addressed by the world space point predicted by the network for the keypoint.
   *
   * \param bucketIndices     An image containing the bucket (example reservoir) indices associated with the keypoints.
   * \param outputPredictions An image into which to store the SCoRe predictions.
   */
  virtual void set_bucket_predictions_for_keypoints(const BucketIndicesImage_CPtr& bucketIndices, ScorePredictionsImage_Ptr& outputPredictions) const = 0;

  /**
   * \brief Sets a SCoRe prediction for each keypoint that contains a single cluster consisting of the world space point
   *        predicted by the network for the keypoint.
   *
   * \param keypointsImage    The image containing the keypoints extracted from the RGB-D image.
   * \param scoreNetOutput    A memory block containing the output tensor produced by the SCoRe network.
   * \param outputPredictions An image into which to store the SCoRe predictions.
   */
  virtual void set_net_predictions_for_keypoints(const Keypoint3DColourImage_CPtr& keypointsImage, const ScoreNetOutput_CPtr& scoreNetOutput,
                                                 ScorePredictionsImage_Ptr& outputPredictions) const = 0;

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /** Override */
  virtual void make_predictions(const ORUChar4Image *colourImage) const;

  /** Override */
  virtual void train_sub(const ORUChar4Image *colourImage, const ORFloatImage *depthImage, const Vector4f& depthIntrinsics, const ORUtils::SE3Pose& cameraPose);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Finds the example reservoir corresponding to each keypoint in the specified colour image.
   *
   * \param colourImage     The colour image for whose keypoints we want to find the example reservoirs.
   * \param allowAllocation Whether or not to allocate new reservoirs if necessary.
   */
  void find_reservoirs(const ORUChar4Image *colourImage, bool allowAllocation) const;

  /**
   * \brief Runs the network on the specified colour image to predict a world space point for each keypoint.
   *
   * \param colourImage The colour image on which to run the network.
   */
  void run_net(const ORUChar4Image *colourImage) const;
};

}

#endif
