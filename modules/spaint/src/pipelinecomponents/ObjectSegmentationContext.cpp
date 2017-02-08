/**
 * spaint: ObjectSegmentationContext.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/ObjectSegmentationContext.h"

#include <tvgutil/containers/MapUtil.h>
using namespace tvgutil;

namespace spaint {

//#################### DESTRUCTOR ####################

ObjectSegmentationContext::~ObjectSegmentationContext() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

ITMUChar4Image_CPtr ObjectSegmentationContext::get_segmentation_image(const std::string& sceneID) const
{
  return MapUtil::lookup(m_segmentationImages, sceneID, ITMUChar4Image_CPtr());
}

boost::optional<tvgutil::SequentialPathGenerator>& ObjectSegmentationContext::get_segmentation_path_generator()
{
  return m_segmentationPathGenerator;
}

const Segmenter_Ptr& ObjectSegmentationContext::get_segmenter() const
{
  return m_segmenter;
}

void ObjectSegmentationContext::set_segmentation_image(const std::string& sceneID, const ITMUChar4Image_CPtr& segmentationImage)
{
  m_segmentationImages[sceneID] = segmentationImage;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void ObjectSegmentationContext::set_segmenter(const Segmenter_Ptr& segmenter)
{
  m_segmenter = segmenter;
}

}
