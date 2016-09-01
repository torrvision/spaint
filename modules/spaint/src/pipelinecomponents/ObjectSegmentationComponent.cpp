/**
 * spaint: ObjectSegmentationComponent.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/ObjectSegmentationComponent.h"
using namespace tvgutil;

#include <boost/serialization/shared_ptr.hpp>

#include "segmentation/SegmentationUtil.h"
#include "util/ImagePersister.h"

#if WITH_ARRAYFIRE && WITH_OPENCV
#include "segmentation/BackgroundSubtractingObjectSegmenter.h"
#endif

namespace spaint {

//#################### CONSTRUCTORS ####################

ObjectSegmentationComponent::ObjectSegmentationComponent(const ObjectSegmentationContext_Ptr& context, const std::string& sceneID)
: m_context(context), m_sceneID(sceneID)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void ObjectSegmentationComponent::reset_segmenter()
{
  const Segmenter_Ptr& segmenter = get_segmenter();
  if(segmenter) segmenter->reset();
}

void ObjectSegmentationComponent::run_segmentation(const VoxelRenderState_CPtr& renderState)
{
  // Gets the current segmenter. If there isn't one, early out.
  const Segmenter_Ptr& segmenter = get_segmenter();
  if(!segmenter) return;

  // Segment the current input images to obtain a mask for the target.
  ITMUCharImage_CPtr targetMask = segmenter->segment(m_context->get_pose(m_sceneID), renderState);

  // If the mask is empty, early out.
  if(!targetMask)
  {
    m_context->set_segmentation_image(ITMUChar4Image_Ptr());
    return;
  }

  // Make masked versions of the depth and colour inputs.
  View_CPtr view = m_context->get_view(m_sceneID);
  ITMUChar4Image_Ptr colouredDepthInput(new ITMUChar4Image(view->depth->dataSize, true, false));
  m_context->get_visualisation_generator()->get_depth_input(colouredDepthInput, view);
  ITMShortImage_Ptr depthInput = m_context->get_input_raw_depth_image_copy(m_sceneID);
  ITMUChar4Image_CPtr rgbInput(m_context->get_view(m_sceneID)->rgb, boost::serialization::null_deleter());

  ITMUChar4Image_CPtr colouredDepthMasked = SegmentationUtil::apply_mask(targetMask, colouredDepthInput);
  ITMShortImage_Ptr depthMasked = SegmentationUtil::apply_mask(targetMask, depthInput);
  ITMUChar4Image_CPtr rgbMasked = SegmentationUtil::apply_mask(targetMask, rgbInput);

  boost::optional<SequentialPathGenerator>& segmentationPathGenerator = m_context->get_segmentation_path_generator();
  if(segmentationPathGenerator)
  {
    // Save the original and masked versions of the depth and colour inputs to disk so that they can be used later for training.
    segmentationPathGenerator->increment_index();
    ImagePersister::save_image_on_thread(colouredDepthInput, segmentationPathGenerator->make_path("cdepth%06i.png"));
    ImagePersister::save_image_on_thread(colouredDepthMasked, segmentationPathGenerator->make_path("cdepthm%06i.png"));
    ImagePersister::save_image_on_thread(depthInput, segmentationPathGenerator->make_path("depth%06i.pgm"));
    ImagePersister::save_image_on_thread(depthMasked, segmentationPathGenerator->make_path("depthm%06i.pgm"));
    ImagePersister::save_image_on_thread(rgbInput, segmentationPathGenerator->make_path("rgb%06i.ppm"));
    ImagePersister::save_image_on_thread(rgbMasked, segmentationPathGenerator->make_path("rgbm%06i.ppm"));
  }

  // Set the masked colour image as the segmentation overlay image so that it will be rendered.
  m_context->set_segmentation_image(rgbMasked);
}

void ObjectSegmentationComponent::run_segmentation_training(const VoxelRenderState_CPtr& renderState)
{
  const Segmenter_Ptr& segmenter = get_segmenter();
  if(!segmenter) return;

  ITMUChar4Image_Ptr touchImage = segmenter->train(m_context->get_pose(m_sceneID), renderState);
  m_context->set_segmentation_image(touchImage);
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

const Segmenter_Ptr& ObjectSegmentationComponent::get_segmenter() const
{
#if WITH_ARRAYFIRE && WITH_OPENCV
  if(!m_context->get_segmenter())
  {
    const TouchSettings_Ptr touchSettings(new TouchSettings(m_context->get_resources_dir() + "/TouchSettings.xml"));
    m_context->set_segmenter(Segmenter_Ptr(new BackgroundSubtractingObjectSegmenter(m_context->get_view(m_sceneID), m_context->get_settings(), touchSettings)));
  }
#endif

  return m_context->get_segmenter();
}

}
