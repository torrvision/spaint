/**
 * itmx: CascadeRelocaliser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_ITMX_CASCADERELOCALISER
#define H_ITMX_CASCADERELOCALISER

#include <boost/optional.hpp>

#include <orx/relocalisation/Relocaliser.h>

#include <tvgutil/filesystem/SequentialPathGenerator.h>
#include <tvgutil/timing/AverageTimer.h>

#include "../base/ITMObjectPtrTypes.h"

namespace itmx {

/**
 * \brief An instance of this class can be used to combine the results of several relocalisers in a cascade.
 */
class CascadeRelocaliser : public orx::Relocaliser
{
  //#################### TYPEDEFS ####################
private:
  typedef tvgutil::AverageTimer<boost::chrono::microseconds> AverageTimer;

  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  /** The "Fast" relocaliser. */
  orx::Relocaliser_Ptr m_innerRelocaliser_Fast;

  /** The "Intermediate" relocaliser. */
  orx::Relocaliser_Ptr m_innerRelocaliser_Intermediate;

  /** The "Slow" relocaliser, also used for training and update. */
  orx::Relocaliser_Ptr m_innerRelocaliser_Full;

  /** The path generator used when saving the relocalised poses. */
  mutable boost::optional<tvgutil::SequentialPathGenerator> m_posePathGenerator;

  /** Whether or not to enable relocalisation with the INTERMEDIATE relocaliser if the results of the fast one are not satisfactory. */
  bool m_relocaliserEnabled_Intermediate;

  /** Whether or not to enable relocalisation with the FULL relocaliser if the results of the previous one are not satisfactory. */
  bool m_relocaliserEnabled_Full;

  /** The score threshold used when deciding if the INTERMEDIATE relocaliser has to be used. */
  float m_relocaliserThresholdScore_Intermediate;

  /** The score threshold used when deciding if the FULL relocaliser has to be used. */
  float m_relocaliserThresholdScore_Full;

  /** Whether or not to save the relocalised poses. */
  bool m_savePoses;

  /** Whether or not to save the average relocalisation times. */
  bool m_saveTimes;

  /** The settings to use for InfiniTAM. */
  Settings_CPtr m_settings;

  /** The timer used to profile the initial relocalisations. */
  mutable AverageTimer m_timerInitialRelocalisation;

  /** The timer used to profile the ICP refinement. */
  mutable AverageTimer m_timerRefinement;

  /** The timer used to profile the relocalisation calls. */
  mutable AverageTimer m_timerRelocalisation;

  /** Whether or not timers are enabled and stats are printed on destruction. */
  bool m_timersEnabled;

  /** The path to a file where to save the average relocalisation times. */
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
   * \param forestPath  The path to the file containing the forest for the relocaliser.
   * \param settings    The settings to use when constructing the relocaliser.
   */
  CascadeRelocaliser(const std::string& forestPath, const Settings_CPtr& settings);

  /**
   * \brief Constructs an ICP-based refining relocaliser.
   *
   * \param innerRelocaliser_Fast          The Fast relocaliser.
   * \param innerRelocaliser_Intermediate  The Intermediate relocaliser.
   * \param innerRelocaliser_Full          The Fast relocaliser.
   * \param settings                       The settings.
   */
  CascadeRelocaliser(const orx::Relocaliser_Ptr& innerRelocaliser_Fast, const orx::Relocaliser_Ptr& innerRelocaliser_Intermediate,
                     const orx::Relocaliser_Ptr& innerRelocaliser_Full, const Settings_CPtr& settings);

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

  /**
   * \brief Starts the specified timer (waiting for all CUDA operations to terminate first, if necessary).
   *
   * \param timer            The timer to start.
   * \param cudaSynchronize  Whether or not to call cudaDeviceSynchronize before starting the timer.
   */
  void start_timer(AverageTimer& timer, bool cudaSynchronize = true) const;

  /**
   * \brief Stops the specified timer (waiting for all CUDA operations to terminate first, if necessary).
   *
   * \param timer The timer to stop.
   * \param cudaSynchronize  Whether or not to call cudaDeviceSynchronize before stopping the timer.
   */
  void stop_timer(AverageTimer& timer, bool cudaSynchronize = true) const;
};

}

#endif
