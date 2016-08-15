/**
 * spaintgui: SLAMSection.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_SLAMSECTION
#define H_SPAINTGUI_SLAMSECTION

#include <boost/shared_ptr.hpp>

#include <InputSource/CompositeImageSourceEngine.h>
#include <ITMLib/Engines/ViewBuilding/Interface/ITMViewBuilder.h>
#include <ITMLib/Objects/RenderStates/ITMRenderState.h>

#include "SLAMState.h"

/**
 * \brief TODO
 */
class SLAMSection
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<InputSource::CompositeImageSourceEngine> CompositeImageSourceEngine_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMDenseMapper<spaint::SpaintVoxel,ITMVoxelIndex> > DenseMapper_Ptr;
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;
  typedef ITMLib::ITMScene<spaint::SpaintVoxel,ITMVoxelIndex> Scene;
  typedef boost::shared_ptr<Scene> Scene_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMLibSettings> Settings_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMViewBuilder> ViewBuilder_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The dense mapper. */
  DenseMapper_Ptr m_denseMapper;

  /** The number of frames for which fusion has been run. */
  size_t m_fusedFramesCount;

  /** The engine used to provide input images to the fusion process. */
  CompositeImageSourceEngine_Ptr m_imageSourceEngine;

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

  /** The remaining number of frames for which we need to achieve good tracking before we can add another keyframe. */
  size_t m_keyframeDelay;

  /** The reconstructed scene. */
  Scene_Ptr m_scene;

  /** The view builder. */
  ViewBuilder_Ptr m_viewBuilder;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief TODO
   *
   * \param imageSourceEngine TODO
   * \param settings          The settings to use for InfiniTAM.
   */
  SLAMSection(const CompositeImageSourceEngine_Ptr& imageSourceEngine, const Settings_Ptr& settings);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  ITMShortImage_CPtr get_input_raw_depth_image() const;

  /**
   * \brief TODO
   */
  ITMUChar4Image_CPtr get_input_rgb_image() const;

  /**
   * \brief TODO
   */
  const Scene_Ptr& get_scene();

  /** TODO */
  virtual bool run(SLAMState& state);
};

#endif
