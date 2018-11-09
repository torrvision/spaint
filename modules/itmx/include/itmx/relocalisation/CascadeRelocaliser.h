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
 * \brief An instance of this class can be used to combine the results of several relocalisers in a cascade.
 */
class CascadeRelocaliser : public orx::Relocaliser
{
  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  /** The "Fast" relocaliser. */
  orx::Relocaliser_Ptr m_innerRelocaliser_Fast;

  /** The "Intermediate" relocaliser. */
  orx::Relocaliser_Ptr m_innerRelocaliser_Intermediate;

  /** The "Slow" relocaliser, also used for training and update. */
  orx::Relocaliser_Ptr m_innerRelocaliser_Slow;

  /** The path generator used when saving the relocalised poses. */
  mutable boost::optional<tvgutil::SequentialPathGenerator> m_posePathGenerator;

  /** Whether or not to enable relocalisation with the "Intermediate" relocaliser if the results of the fast one are not satisfactory. */
  bool m_relocaliserEnabled_Intermediate;

  /** Whether or not to enable relocalisation with the "Slow" relocaliser if the results of the previous one are not satisfactory. */
  bool m_relocaliserEnabled_Slow;

  /** The score threshold used when deciding if the "Intermediate" relocaliser has to be used. */
  float m_relocaliserThresholdScore_Intermediate;

  /** The score threshold used when deciding if the "Slow" relocaliser has to be used. */
  float m_relocaliserThresholdScore_Slow;

  /** Whether or not to save the relocalised poses. */
  bool m_savePoses;

  /** Whether or not to save the average relocalisation times. */
  bool m_saveTimes;

  /** The settings to use for the relocaliser. */
  Settings_CPtr m_settings;

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
   * \param innerRelocaliser_Fast          The "Fast" relocaliser.
   * \param innerRelocaliser_Intermediate  The "Intermediate" relocaliser.
   * \param innerRelocaliser_Slow          The "Slow" relocaliser.
   * \param settings                       The settings to use for the relocaliser.
   */
  CascadeRelocaliser(const orx::Relocaliser_Ptr& innerRelocaliser_Fast, const orx::Relocaliser_Ptr& innerRelocaliser_Intermediate,
                     const orx::Relocaliser_Ptr& innerRelocaliser_Slow, const Settings_CPtr& settings);

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
  virtual std::vector<Result> relocalise(const ORUChar4Image *colourImage, const ORFloatImage *depthImage,
                                         const Vector4f& depthIntrinsics) const;

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
