/**
 * spaintgui: PipelineState.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_PIPELINESTATE
#define H_SPAINTGUI_PIPELINESTATE

#include <InputSource/CompositeImageSourceEngine.h>
#include <ITMLib/Core/ITMDenseMapper.h>
#include <ITMLib/Core/ITMTrackingController.h>
#include <ITMLib/Engines/LowLevel/Interface/ITMLowLevelEngine.h>
#include <ITMLib/Engines/ViewBuilding/Interface/ITMViewBuilder.h>
#include <ITMLib/Objects/Misc/ITMIMUCalibrator.h>
#include <RelocLib/PoseDatabase.h>
#include <RelocLib/Relocaliser.h>

#include <rafl/core/RandomForest.h>

#include <spaint/features/interface/FeatureCalculator.h>
#include <spaint/propagation/interface/LabelPropagator.h>
#include <spaint/sampling/interface/PerLabelVoxelSampler.h>
#include <spaint/sampling/interface/UniformVoxelSampler.h>
#include <spaint/smoothing/interface/LabelSmoother.h>
#include <spaint/trackers/FallibleTracker.h>
#include <spaint/util/SpaintVoxel.h>

#include "FeatureInspectionState.h"
#include "Interactor.h"
#include "PipelineMode.h"
#include "PredictionState.h"
#include "PropagationState.h"
#include "Raycaster.h"
#include "SmoothingState.h"
#include "TrackerType.h"
#include "TrainingState.h"

/**
 * \brief An instance of this class represents the state shared between the different sections of the spaintgui processing pipeline.
 */
class PipelineState
: public FeatureInspectionState,
  public PredictionState,
  public PropagationState,
  public SmoothingState,
  public TrainingState
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<InputSource::CompositeImageSourceEngine> CompositeImageSourceEngine_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMDenseMapper<spaint::SpaintVoxel,ITMVoxelIndex> > DenseMapper_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMIMUCalibrator> IMUCalibrator_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMTracker> ITMTracker_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMLowLevelEngine> LowLevelEngine_Ptr;
  typedef boost::shared_ptr<RelocLib::PoseDatabase> PoseDatabase_Ptr;
  typedef boost::shared_ptr<rafl::RandomForest<spaint::SpaintVoxel::Label> > RandomForest_Ptr;
  typedef boost::shared_ptr<const rafl::RandomForest<spaint::SpaintVoxel::Label> > RandomForest_CPtr;
  typedef boost::shared_ptr<RelocLib::Relocaliser> Relocaliser_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMTrackingController> TrackingController_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMViewBuilder> ViewBuilder_Ptr;

  //#################### PUBLIC VARIABLES ####################
public:
  /** The dense mapper. */
  DenseMapper_Ptr m_denseMapper;

  /** A pointer to a tracker that can detect tracking failures (if available). */
  spaint::FallibleTracker *m_fallibleTracker;

  /** The feature calculator. */
  spaint::FeatureCalculator_CPtr m_featureCalculator;

  /** The name to give the feature inspection window. */
  std::string m_featureInspectionWindowName;

  /** The random forest. */
  RandomForest_Ptr m_forest;

  /** The number of frames for which fusion has been run. */
  size_t m_fusedFramesCount;

  /** Whether or not the user wants fusion to be run as part of the pipeline. */
  bool m_fusionEnabled;

  /** The engine used to provide input images to the fusion pipeline. */
  CompositeImageSourceEngine_Ptr m_imageSourceEngine;

  /** The IMU calibrator. */
  IMUCalibrator_Ptr m_imuCalibrator;

  /**
   * A number of initial frames to fuse, regardless of their tracking quality.
   * Tracking quality can be poor in the first few frames, when there is only
   * a limited model against which to track. By forcibly fusing these frames,
   * we prevent poor tracking quality from stopping the reconstruction. After
   * these frames have been fused, only frames with a good tracking result will
   * be fused.
   */
  size_t m_initialFramesToFuse;

  /** The image into which depth input is read each frame. */
  ITMShortImage_Ptr m_inputRawDepthImage;

  /** The image into which RGB input is read each frame. */
  ITMUChar4Image_Ptr m_inputRGBImage;

  /** The interactor that is used to interact with the InfiniTAM scene. */
  Interactor_Ptr m_interactor;

  /** The remaining number of frames for which we need to achieve good tracking before we can add another keyframe. */
  size_t m_keyframeDelay;

  /** The label propagator. */
  spaint::LabelPropagator_CPtr m_labelPropagator;

  /** The label smoother. */
  spaint::LabelSmoother_CPtr m_labelSmoother;

  /** The engine used to perform low-level image processing operations. */
  LowLevelEngine_Ptr m_lowLevelEngine;

  /** The maximum number of voxels for which to predict labels each frame. */
  size_t m_maxPredictionVoxelCount;

  /** The maximum number of voxels per label from which to train each frame. */
  size_t m_maxTrainingVoxelsPerLabel;

  /** The mode in which the pipeline is currently running. */
  PipelineMode m_mode;

  /** The spaint model. */
  Model_Ptr m_model;

  /** The side length of a VOP patch (must be odd). */
  size_t m_patchSize;

  /** The database of previous poses for relocalisation. */
  PoseDatabase_Ptr m_poseDatabase;

  /** A memory block in which to store the feature vectors computed for the various voxels during prediction. */
  boost::shared_ptr<ORUtils::MemoryBlock<float> > m_predictionFeaturesMB;

  /** A memory block in which to store the labels predicted for the various voxels. */
  boost::shared_ptr<ORUtils::MemoryBlock<spaint::SpaintVoxel::PackedLabel> > m_predictionLabelsMB;

  /** The voxel sampler used in prediction mode. */
  spaint::UniformVoxelSampler_CPtr m_predictionSampler;

  /** A memory block in which to store the locations of the voxels sampled for prediction purposes. */
  spaint::Selector::Selection_Ptr m_predictionVoxelLocationsMB;

  /** The raycaster that is used to cast rays into the InfiniTAM scene. */
  Raycaster_Ptr m_raycaster;

  /** The relocaliser. */
  Relocaliser_Ptr m_relocaliser;

  /** The path to the resources directory. */
  std::string m_resourcesDir;

  /** The tracker. */
  ITMTracker_Ptr m_tracker;

  /**
   * The parameters for the tracker (if any). For example, this would be the host on which the
   * Vicon software is running (e.g. "<IP address>:<port>") if we're using the Vicon tracker.
   */
  std::string m_trackerParams;

  /** The type of tracker to use. */
  TrackerType m_trackerType;

  /** The tracking controller. */
  TrackingController_Ptr m_trackingController;

  /** A memory block in which to store the feature vectors computed for the various voxels during training. */
  boost::shared_ptr<ORUtils::MemoryBlock<float> > m_trainingFeaturesMB;

  /** A memory block in which to store a mask indicating which labels are currently in use and from which we want to train. */
  boost::shared_ptr<ORUtils::MemoryBlock<bool> > m_trainingLabelMaskMB;

  /** The voxel sampler used in training mode. */
  spaint::PerLabelVoxelSampler_CPtr m_trainingSampler;

  /** A memory block in which to store the number of voxels sampled for each label for training purposes. */
  boost::shared_ptr<ORUtils::MemoryBlock<unsigned int> > m_trainingVoxelCountsMB;

  /** A memory block in which to store the locations of the voxels sampled for training purposes. */
  spaint::Selector::Selection_Ptr m_trainingVoxelLocationsMB;

  /** The view builder. */
  ViewBuilder_Ptr m_viewBuilder;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual const spaint::FeatureCalculator_CPtr& get_feature_calculator() const
  {
    return m_featureCalculator;
  }

  /** Override */
  virtual const std::string& get_feature_inspection_window_name() const
  {
    return m_featureInspectionWindowName;
  }

  /** Override */
  virtual const RandomForest_Ptr& get_forest()
  {
    return m_forest;
  }

  /** Override */
  virtual const Interactor_Ptr& get_interactor() const
  {
    return m_interactor;
  }

  /** Override */
  virtual const spaint::LabelPropagator_CPtr& get_label_propagator() const
  {
    return m_labelPropagator;
  }

  /** Override */
  virtual const spaint::LabelSmoother_CPtr& get_label_smoother() const
  {
    return m_labelSmoother;
  }

  /** Override */
  virtual size_t get_max_prediction_voxel_count() const
  {
    return m_maxPredictionVoxelCount;
  }

  /** Override */
  virtual size_t get_max_training_voxels_per_label() const
  {
    return m_maxTrainingVoxelsPerLabel;
  }

  /** Override */
  virtual const Model_Ptr& get_model() const
  {
    return m_model;
  }

  /** Override */
  virtual size_t get_patch_size() const
  {
    return m_patchSize;
  }

  /** Override */
  virtual const boost::shared_ptr<ORUtils::MemoryBlock<float> >& get_prediction_features()
  {
    return m_predictionFeaturesMB;
  }

  /** Override */
  virtual const boost::shared_ptr<ORUtils::MemoryBlock<spaint::SpaintVoxel::PackedLabel> >& get_prediction_labels()
  {
    return m_predictionLabelsMB;
  }

  /** Override */
  virtual const spaint::UniformVoxelSampler_CPtr& get_prediction_sampler() const
  {
    return m_predictionSampler;
  }

  /** Override */
  virtual const spaint::Selector::Selection_Ptr& get_prediction_voxel_locations()
  {
    return m_predictionVoxelLocationsMB;
  }

  /** Override */
  virtual const boost::shared_ptr<ORUtils::MemoryBlock<float> >& get_training_features() const
  {
    return m_trainingFeaturesMB;
  }

  /** Override */
  virtual const boost::shared_ptr<ORUtils::MemoryBlock<bool> >& get_training_label_mask() const
  {
    return m_trainingLabelMaskMB;
  }

  /** Override */
  virtual const spaint::PerLabelVoxelSampler_CPtr& get_training_sampler() const
  {
    return m_trainingSampler;
  }

  /** Override */
  virtual const boost::shared_ptr<ORUtils::MemoryBlock<unsigned int> >& get_training_voxel_counts() const
  {
    return m_trainingVoxelCountsMB;
  }

  /** Override */
  virtual const spaint::Selector::Selection_Ptr& get_training_voxel_locations()
  {
    return m_trainingVoxelLocationsMB;
  }
};

#endif
