/**
 * spaint: ObjectSegmentationContext.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/ObjectSegmentationContext.h"

namespace spaint {

//#################### DESTRUCTOR ####################

ObjectSegmentationContext::~ObjectSegmentationContext() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

const ITMUChar4Image_CPtr& ObjectSegmentationContext::get_segmentation_image() const
{
  return m_segmentationImage;
}

boost::optional<tvgutil::SequentialPathGenerator>& ObjectSegmentationContext::get_segmentation_path_generator()
{
  return m_segmentationPathGenerator;
}

const Segmenter_Ptr& ObjectSegmentationContext::get_segmenter() const
{
  return m_segmenter;
}

void ObjectSegmentationContext::set_segmentation_image(const ITMUChar4Image_CPtr& segmentationImage)
{
  m_segmentationImage = segmentationImage;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void ObjectSegmentationContext::set_segmenter(const Segmenter_Ptr& segmenter)
{
  m_segmenter = segmenter;
}

}
