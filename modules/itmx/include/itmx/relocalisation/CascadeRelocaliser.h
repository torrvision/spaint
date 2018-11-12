/**
 * itmx: CascadeRelocaliser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_ITMX_CASCADERELOCALISER
#define H_ITMX_CASCADERELOCALISER

#include <boost/optional.hpp>

#include <orx/relocalisation/Relocaliser.h>

#include <tvgutil/filesystem/SequentialPathGenerator.h>

#include "../base/ITMObjectPtrTypes.h"

namespace itmx {

/**
 * \brief An instance of this class represents a cascade relocaliser that gradually falls back from faster, weaker relocalisers to slower, stronger ones.
 */
class CascadeRelocaliser : public orx::Relocaliser
{
  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  /** The thresholds used to decide whether or not to fall back from one relocaliser in the cascade to the next. */
  std::vector<float> m_fallbackThresholds;

  /** The individual relocalisers in the cascade. */
  std::vector<orx::Relocaliser_Ptr> m_innerRelocalisers;

  /** The path generator used when saving the relocalised poses. */
  mutable boost::optional<tvgutil::SequentialPathGenerator> m_posePathGenerator;

  /** Whether or not to save the relocalised poses. */
  bool m_savePoses;

  /** Whether or not to save the average relocalisation times. */
  bool m_saveTimes;

  /** The timer used to profile the initial relocalisations. */
  mutable AverageTimer m_timerInitialRelocalisation;

  /** The timer used to profile the ICP refinement. */
  mutable AverageTimer m_timerRefinement;

  /** The timer used to profile the relocalisation calls. */
  mutable AverageTimer m_timerRelocalisation;

  /** The path to a file in which to save the average relocalisation times. */
  std::string m_timersOutputFile;

  /** The timer used to profile the training calls. */
  AverageTimer m_timerTraining;

  /** The timer used to profile the update calls. */
  AverageTimer m_timerUpdate;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a cascade relocaliser.
   *
   * \param innerRelocalisers The individual relocalisers in the cascade.
   * \param settings          The settings to use.
   */
  CascadeRelocaliser(const std::vector<orx::Relocaliser_Ptr>& innerRelocalisers, const Settings_CPtr& settings);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the relocaliser.
   */
  ~CascadeRelocaliser();

  //#################### COPY CONSTRUCTOR & ASSIGNMENT OPERATOR ####################
private:
  // Deliberately private and unimplemented.
  CascadeRelocaliser(const CascadeRelocaliser&);
  CascadeRelocaliser& operator=(const CascadeRelocaliser&);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void finish_training();

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

  /** Override */
  virtual void update();

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Saves the relocalised and refined poses in text files so that they can be used later (e.g. for evaluation).
   *
   * \note Saving happens only if m_savePoses is true.
   *
   * \param relocalisedPose The relocalised pose before refinement.
   * \param refinedPose     The result of refining the relocalised pose.
   */
  void save_poses(const Matrix4f& relocalisedPose, const Matrix4f& refinedPose) const;
};

}

#endif
