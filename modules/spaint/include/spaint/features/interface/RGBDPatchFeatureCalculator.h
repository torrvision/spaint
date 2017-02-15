/**
 * spaint: RGBDPatchFeatureCalculator.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_RGBDPATCHFEATURECALCULATOR
#define H_SPAINT_RGBDPATCHFEATURECALCULATOR

#include <boost/shared_ptr.hpp>

#include "RGBDPatchFeature.h"
#include "../../util/ITMImagePtrTypes.h"
#include "../../util/ITMMemoryBlockPtrTypes.h"

namespace spaint
{

/**
 * \brief An instance of a class deriving from this one can be used to compute
 *        features based on depth and colour differences in RGBD images.
 *
 * The features are computed as described by Valentin et al. in "Exploiting
 * Uncertainty in Regression Forests for Accurate Camera Relocalization".
 */
class RGBDPatchFeatureCalculator
{
  //#################### PROTECTED MEMBER VARIABLES ####################
protected:
  /** A memory block storing the colour channels associated to the RGB part of the descriptor. */
  ITMUCharMemoryBlock_Ptr m_channelsRgb;

  /** The step used to sample keypoints from the image. */
  uint32_t m_featureStep;

  /** Whether or not to normalize depth offsets by the depth associated to the corresponding keypoint. */
  bool m_normalizeDepth;

  /** Whether or not to normalize RGB offsets by the depth associated to the corresponding keypoint. */
  bool m_normalizeRgb;

  /** A memory block storing the offsets used to sample the depth values used in the descriptor. */
  ITMInt4MemoryBlock_Ptr m_offsetsDepth;

  /** A memory block storing the offsets used to sample the colour pixels used in the descriptor. */
  ITMInt4MemoryBlock_Ptr m_offsetsRgb;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an RGBD patch feature calculator.
   */
  RGBDPatchFeatureCalculator();

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the RGBD patch feature calculator.
   */
  virtual ~RGBDPatchFeatureCalculator();

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Extract keypoints and compute the feature for an RGBD image.
   *        Each keypoint position is in world coordinates, obtained chaining
   *        the cameraPose transformation with the 3D coordinates of the point
   *        in camera coordinates.
   *
   * \note  The implicit assumption is that pixels in the colour image and in
   *        the depth image are registered.
   * \note  Keypoint/Descriptors are computed on a grid in the input image pair,
   *        according to the step set by set_feature_step.
   *
   * \param rgbImage        The colour image.
   * \param depthImage      The depth image.
   * \param intrinsics      The intrinsic parameters of the depth camera.
   * \param keypointsImage  The output image that will contain the 3D coordinates
   *                        (in world reference frame, see cameraPose) and colour
   *                        of the extracted features. Will be resized as necessary.
   * \param featuresImage   The output image that will contain the descriptors.
   * \param cameraPose      A transformation from the camera reference frame to a
   *                        world reference frame that will be applied to the 3D
   *                        keypoint positions.
   */
  virtual void compute_feature(const ITMUChar4Image *rgbImage, const ITMFloatImage *depthImage, const Vector4f &intrinsics,
                               Keypoint3DColourImage *keypointsImage, RGBDPatchDescriptorImage *featuresImage, const Matrix4f &cameraPose) const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Extract keypoints and compute the feature for an RGBD image.
   *        Each keypoint position is in camera coordinates, according to
   *        the depth camera intrinsics.
   *
   * \note  The implicit assumption is that pixels in the colour image and in
   *        the depth image are registered.
   * \note  Keypoint/Descriptors are computed on a grid in the input image pair,
   *        according to the step set by set_feature_step.
   *
   * \param rgbImage        The colour image.
   * \param depthImage      The depth image.
   * \param intrinsics      The intrinsic parameters of the depth camera.
   * \param keypointsImage  The output image that will contain the 3D coordinates
   *                        (in camera reference frame) and colour
   *                        of the extracted features. Will be resized as necessary.
   * \param featuresImage   The output image that will contain the descriptors.
   */
  void compute_feature(const ITMUChar4Image *rgbImage, const ITMFloatImage *depthImage, const Vector4f &intrinsics,
                       Keypoint3DColourImage *keypointsImage, RGBDPatchDescriptorImage *featuresImage) const;

  /**
   * \brief Gets the step used when selecting keypoints and computing the features.
   *
   * \return  The step used when selecting keypoints and computing the features.
   */
  uint32_t get_feature_step() const;

  /**
   * \brief Sets the step used when selecting keypoints and computing the features.
   *
   * \param featureStep The step used when selecting keypoints and computing the features.
   */
  void set_feature_step(uint32_t featureStep);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<RGBDPatchFeatureCalculator> RGBDPatchFeatureCalculator_Ptr;
typedef boost::shared_ptr<const RGBDPatchFeatureCalculator> RGBDPatchFeatureCalculator_CPtr;

}

#endif
