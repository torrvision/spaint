/**
 * grove: CPUInstantiations.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "features/FeatureCalculatorFactory.tpp"
#include "features/cpu/RGBDPatchFeatureCalculator_CPU.tpp"
#include "features/interface/RGBDPatchFeatureCalculator.tpp"

namespace grove {

//#################### EXPLICIT INSTANTIATIONS ####################

template boost::shared_ptr<RGBDPatchFeatureCalculator<Keypoint2D,RGBDPatchDescriptor> >
  FeatureCalculatorFactory::make_custom_patch_feature_calculator<Keypoint2D,RGBDPatchDescriptor>(
    ITMLib::ITMLibSettings::DeviceType deviceType,
    bool depthAdaptive,
    RGBDPatchFeatureCalculatorDifferenceType depthDifferenceType,
    uint32_t depthFeatureCount,
    uint32_t depthFeatureOffset,
    uint32_t depthMinRadius,
    uint32_t depthMaxRadius,
    RGBDPatchFeatureCalculatorDifferenceType rgbDifferenceType,
    uint32_t rgbFeatureCount,
    uint32_t rgbFeatureOffset,
    uint32_t rgbMinRadius,
    uint32_t rgbMaxRadius);
template boost::shared_ptr<RGBDPatchFeatureCalculator<Keypoint3DColour,RGBDPatchDescriptor> >
  FeatureCalculatorFactory::make_custom_patch_feature_calculator<Keypoint3DColour,RGBDPatchDescriptor>(
    ITMLib::ITMLibSettings::DeviceType deviceType,
    bool depthAdaptive,
    RGBDPatchFeatureCalculatorDifferenceType depthDifferenceType,
    uint32_t depthFeatureCount,
    uint32_t depthFeatureOffset,
    uint32_t depthMinRadius,
    uint32_t depthMaxRadius,
    RGBDPatchFeatureCalculatorDifferenceType rgbDifferenceType,
    uint32_t rgbFeatureCount,
    uint32_t rgbFeatureOffset,
    uint32_t rgbMinRadius,
    uint32_t rgbMaxRadius);
template class RGBDPatchFeatureCalculator<Keypoint2D,RGBDPatchDescriptor>;
template class RGBDPatchFeatureCalculator<Keypoint3DColour,RGBDPatchDescriptor>;
template class RGBDPatchFeatureCalculator_CPU<Keypoint2D,RGBDPatchDescriptor>;
template class RGBDPatchFeatureCalculator_CPU<Keypoint3DColour,RGBDPatchDescriptor>;

}
