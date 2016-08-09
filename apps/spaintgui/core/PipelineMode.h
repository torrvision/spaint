/**
 * spaintgui: PipelineMode.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_PIPELINEMODE
#define H_SPAINTGUI_PIPELINEMODE

/**
 * \brief The different modes in which the pipeline can be running.
 */
enum PipelineMode
{
  /** In feature inspection mode, the user can move the mouse around and visualise the features at particular points in the scene. */
  PIPELINEMODE_FEATURE_INSPECTION,

  /** In normal mode, the user can reconstruct and manually label the scene. */
  PIPELINEMODE_NORMAL,

  /** In prediction mode, the random forest is used to predict labels for previously-unseen voxels. */
  PIPELINEMODE_PREDICTION,

  /** In propagation mode, labels supplied by the user are propagated across surfaces in the scene. */
  PIPELINEMODE_PROPAGATION,

  /** In smoothing mode, voxel labels are filled in based on the labels of neighbouring voxels. */
  PIPELINEMODE_SMOOTHING,

  /** In train-and-predict mode, we alternate training and prediction to achieve a pleasing interactive effect. */
  PIPELINEMODE_TRAIN_AND_PREDICT,

  /** In training mode, a random forest is trained using voxels sampled from the current raycast. */
  PIPELINEMODE_TRAINING
};

#endif
