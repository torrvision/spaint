/**
 * itmx: FernRelocaliser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_FERNRELOCALISER
#define H_ITMX_FERNRELOCALISER

#include <FernRelocLib/Relocaliser.h>

#include "Relocaliser.h"

namespace itmx {

/**
 * \brief An instance of this class can be used to relocalise a camera in a 3D scene with the random fern-based relocaliser in InfiniTAM.
 */
class FernRelocaliser : public Relocaliser
{
  //#################### ENUMERATIONS ####################
public:
  /**
   * \brief The values of this enumeration indicate whether to always try to add a keyframe to the pose database,
   *        or to wait after a relocalisation is performed. The latter mode can be used to delay storing keyframes
   *        after a tracking failure until we are sure that they are actually good.
   */
  enum KeyframeAddPolicy
  {
    ALWAYS_TRY_ADD,
    DELAY_AFTER_RELOCALISATION
  };

  //#################### TYPEDEFS ####################
private:
  typedef FernRelocLib::Relocaliser<float> WrappedRelocaliser;
  typedef boost::shared_ptr<WrappedRelocaliser> WrappedRelocaliser_Ptr;

  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  /** The number of decisions to perform in each fern. */
  int m_decisionsPerFern;

  /** The size of the input depth images. */
  Vector2i m_depthImageSize;

  /** The threshold used when deciding whether to store a keyframe. */
  float m_harvestingThreshold;

  /** The policy used to decide whether to store keyframes right after tracking failures. */
  KeyframeAddPolicy m_keyframeAddPolicy;

  /** The delay before trying to add another keyframe to the fern conservatory. */
  mutable uint32_t m_keyframeDelay;

  /** The number of ferns to use for relocalisation. */
  int m_numFerns;

  /** The minimum and maximum range of the depth images. */
  Vector2f m_rangeParameters;

  /** The wrapped relocaliser. */
  WrappedRelocaliser_Ptr m_relocaliser;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a fern relocaliser.
   *
   * \param depthImageSize      The dimensions of the depth images that will be passed to the relocaliser.
   * \param viewFrustumMin      The minimum distance to consider in the depth images.
   * \param viewFrustumMax      The maximum distance to consider in the depth images.
   * \param harvestingThreshold The threshold used when deciding whether to store a keyframe.
   * \param numFerns            The number of ferns to use for relocalisation.
   * \param decisionsPerFern    The number of decisions to perform in each fern.
   * \param keyframeAddPolicy   The policy used to decide whether to store keyframes right after tracking failures.
   */
  FernRelocaliser(const Vector2i& depthImageSize, float viewFrustumMin, float viewFrustumMax,
                  float harvestingThreshold, int numFerns, int decisionsPerFern,
                  KeyframeAddPolicy keyframeAddPolicy = DELAY_AFTER_RELOCALISATION);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void load_from_disk(const std::string& inputFolder);

  /** Override */
  virtual std::vector<Result> relocalise(const ORUChar4Image *colourImage, const ORFloatImage *depthImage, const Vector4f& depthIntrinsics) const;

  /** Override */
  virtual void reset();

  /** Override */
  virtual void save_to_disk(const std::string& outputFolder) const;

  /** Override */
  virtual void train(const ORUChar4Image *colourImage, const ORFloatImage *depthImage,
                     const Vector4f& depthIntrinsics, const ORUtils::SE3Pose& cameraPose);

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the default threshold used when deciding whether to store a keyframe.
   *
   * \return  The default threshold used when deciding whether to store a keyframe.
   */
  static float get_default_harvesting_threshold();

  /**
   * \brief Gets the default policy used to decide whether to store keyframes right after tracking failures.
   *
   * \return  The default policy used to decide whether to store keyframes right after tracking failures.
   */
  static KeyframeAddPolicy get_default_keyframe_add_policy();

  /**
   * \brief Gets the default number of decisions to perform in each fern.
   *
   * \return  The default number of decisions to perform in each fern.
   */
  static int get_default_num_decisions_per_fern();

  /**
   * \brief Gets the default number of ferns to use for relocalisation.
   *
   * \return  The default number of ferns to use for relocalisation.
   */
  static int get_default_num_ferns();
};

}

#endif
