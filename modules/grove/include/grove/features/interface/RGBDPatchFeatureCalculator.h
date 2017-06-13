/**
 * grove: RGBDPatchFeatureCalculator.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_RGBDPATCHFEATURECALCULATOR
#define H_GROVE_RGBDPATCHFEATURECALCULATOR

#include <itmx/base/ITMImagePtrTypes.h>
#include <itmx/base/ITMMemoryBlockPtrTypes.h>

#include <tvgutil/numbers/RandomNumberGenerator.h>

#include "../base/Descriptor.h"
#include "../base/RGBDPatchFeatureDifferenceType.h"
#include "../../keypoints/Keypoint2D.h"
#include "../../keypoints/Keypoint3DColour.h"

namespace grove {

/**
 * \brief An instance of a class deriving from this one can be used to compute features based on depth and colour differences in RGBD images.
 *
 * The features are computed as described in "Exploiting Uncertainty in Regression Forests for Accurate Camera Relocalization".
 *
 * \param KeypointType    The type of keypoint computed by this class.
 * \param DescriptorType  The type of descriptor computed by this class.
 */
template <typename KeypointType, typename DescriptorType>
class RGBDPatchFeatureCalculator
{
  //#################### TYPEDEFS ####################
public:
  typedef ORUtils::Image<DescriptorType> DescriptorsImage;
  typedef ORUtils::Image<KeypointType> KeypointsImage;

  //#################### PROTECTED MEMBER VARIABLES ####################
protected:
  /** The type of difference to use to compute depth features. */
  RGBDPatchFeatureDifferenceType m_depthDifferenceType;

  /** The number of features to compute from the depth image. */
  uint32_t m_depthFeatureCount;

  /** The offset in the descriptor after which we store the depth features. */
  uint32_t m_depthFeatureOffset;

  /** The maximum radius (in pixels, possibly scaled when evaluating the features if m_normaliseDepth is true) used to generate depth offsets. */
  uint32_t m_depthMaxRadius;

  /** The minimum radius (in pixels, possibly scaled when evaluating the features if m_normaliseDepth is true) used to generate depth offsets. */
  uint32_t m_depthMinRadius;

  /**
   * A memory block storing the offsets used to sample the depth values used in the descriptor.
   * There are two offsets involved per depth feature, so we encode them in a Vector4i.
   */
  ITMInt4MemoryBlock_Ptr m_depthOffsets;

  /** The step used to sample keypoints from the image. */
  uint32_t m_featureStep;

  /** Whether or not to normalise depth offsets by the depth associated to the corresponding keypoint. */
  bool m_normaliseDepth;

  /** Whether or not to normalise RGB offsets by the depth associated to the corresponding keypoint. */
  bool m_normaliseRgb;

  /** A memory block storing the colour channels associated with the RGB part of the descriptor. */
  ITMUCharMemoryBlock_Ptr m_rgbChannels;

  /** The type of difference to use to compute RGB features. */
  RGBDPatchFeatureDifferenceType m_rgbDifferenceType;

  /** The number of features to compute from the RGB image. */
  uint32_t m_rgbFeatureCount;

  /** The offset in the descriptor after which we store the RGB features. */
  uint32_t m_rgbFeatureOffset;

  /** The maximum radius (in pixels, possibly scaled when evaluating the features if m_normaliseDepth is true) used to generate colour offsets. */
  uint32_t m_rgbMaxRadius;

  /** The minimum radius (in pixels, possibly scaled when evaluating the features if m_normaliseDepth is true) used to generate colour offsets. */
  uint32_t m_rgbMinRadius;

  /**
   * A memory block storing the offsets used to sample the colour pixels used in the descriptor.
   * There are two offsets involved per colour feature, so we encode them in a Vector4i.
   */
  ITMInt4MemoryBlock_Ptr m_rgbOffsets;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs an RGBD patch feature calculator.
   *
   * \note This is protected to force clients to make use of FeatureCalculatorFactory, which knows the correct values to use for the arguments.
   *
   * \param depthAdaptive        Whether or not to compute the depth-normalised versions of the features.
   * \param depthDifferenceType  The type of difference to use to compute depth features.
   * \param depthFeatureCount    The number of features to compute from the depth image.
   * \param depthFeatureOffset   The offset in the descriptor after which we store the depth features.
   * \param depthMinRadius       The minimum radius used to generate depth offsets.
   * \param depthMaxRadius       The maximum radius used to generate depth offsets.
   * \param rgbDifferenceType    The type of difference to use to compute RGB features.
   * \param rgbFeatureCount      The number of features to compute from the RGB image.
   * \param rgbFeatureOffset     The offset in the descriptor after which we store the RGB features.
   * \param rgbMinRadius         The minimum radius used to generate colour offsets.
   * \param rgbMaxRadius         The maximum radius used to generate colour offsets.
   *
   * \throws std::invalid_argument If depthFeatureCount + rgbFeatureCount > DescriptorType::FEATURE_COUNT, or if the offsets would cause out-of-bounds access.
   */
  RGBDPatchFeatureCalculator(bool depthAdaptive, RGBDPatchFeatureDifferenceType depthDifferenceType,
                             uint32_t depthFeatureCount, uint32_t depthFeatureOffset, uint32_t depthMinRadius,
                             uint32_t depthMaxRadius, RGBDPatchFeatureDifferenceType rgbDifferenceType,
                             uint32_t rgbFeatureCount, uint32_t rgbFeatureOffset, uint32_t rgbMinRadius, uint32_t rgbMaxRadius);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the RGBD patch feature calculator.
   */
  virtual ~RGBDPatchFeatureCalculator();

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Extracts keypoints from an RGBD image and computes feature descriptors for them.
   *        Each keypoint position is in world coordinates, obtained by applying the cameraPose
   *        transformation to the 3D coordinates of the point in camera coordinates.
   *
   * \note  We implicitly assume that pixels in the colour and depth images are registered.
   * \note  Keypoints/descriptors are computed on a grid in the input image pair, with the step
   *        set by set_feature_step.
   *
   * \param rgbImage         The colour image.
   * \param depthImage       The depth image.
   * \param cameraPose       A transformation from the camera's reference frame to the
   *                         world reference frame (this will be applied to the 3D
   *                         keypoint positions in camera coordinates).
   * \param intrinsics       The intrinsic parameters of the depth camera.
   * \param keypointsImage   The output image that will contain the 3D coordinates
   *                         (in the world reference frame, see cameraPose) and colour
   *                         of the extracted keypoints. Will be resized as necessary.
   * \param descriptorsImage The output image that will contain the feature descriptors.
   */
  virtual void compute_keypoints_and_features(const ITMUChar4Image *rgbImage, const ITMFloatImage *depthImage,
                                              const Matrix4f& cameraPose, const Vector4f& intrinsics,
                                              KeypointsImage *keypointsImage, DescriptorsImage *descriptorsImage) const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Extracts keypoints from an RGBD image and computes feature descriptors for them.
   *        Each keypoint position is in camera coordinates, based on the depth camera's intrinsics.
   *
   * \note  We implicitly assume that pixels in the colour and depth images are registered.
   * \note  Keypoints/descriptors are computed on a grid in the input image pair, with the step
   *        set by set_feature_step.
   *
   * \param rgbImage         The colour image.
   * \param depthImage       The depth image.
   * \param intrinsics       The intrinsic parameters of the depth camera.
   * \param keypointsImage   The output image that will contain the 3D coordinates
   *                         (in the camera's reference frame) and colour of the
   *                         extracted features. Will be resized as necessary.
   * \param descriptorsImage The output image that will contain the feature descriptors.
   */
  void compute_keypoints_and_features(const ITMUChar4Image *rgbImage, const ITMFloatImage *depthImage, const Vector4f& intrinsics,
                                      KeypointsImage *keypointsImage, DescriptorsImage *descriptorsImage) const;

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

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Computes the size of feature image to generate.
   *
   * \note This also checks whether or not the input images needed to compute the desired features are available.
   *
   * \param rgbImage    The colour image.
   * \param depthImage  The depth image.
   * \param inputDims   Will be filled with the dimensions of either the colour or depth image, depending on the configuration.
   * \return            The size of feature image to generate.
   *
   * \throws std::invalid_argument If the features cannot be computed.
   */
  Vector2i compute_output_dims(const ITMUChar4Image *rgbImage, const ITMFloatImage *depthImage, Vector2i& inputDims) const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Sets up the colour features.
   */
  void setup_colour_features();

  /**
   * \brief Sets up the depth features.
   */
  void setup_depth_features();

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Generates a random integer offset in the intervals [-max, -min] and [min, max]
   *        using the specified random number generator.
   *
   * \param rng The random number generator to use.
   * \param min The minimum value of the positive interval.
   * \param max The maximum value of the positive interval.
   * \return    A random integer in [-max, -min] U [min, max].
   */
  static int generate_offset(tvgutil::RandomNumberGenerator& rng, int min, int max);
};

//#################### TYPEDEFS ####################

typedef Descriptor<256> RGBDPatchDescriptor;

typedef ORUtils::Image<RGBDPatchDescriptor> RGBDPatchDescriptorImage;
typedef boost::shared_ptr<RGBDPatchDescriptorImage> RGBDPatchDescriptorImage_Ptr;
typedef boost::shared_ptr<const RGBDPatchDescriptorImage> RGBDPatchDescriptorImage_CPtr;

typedef RGBDPatchFeatureCalculator<Keypoint2D,RGBDPatchDescriptor> RGBPatchFeatureCalculator;
typedef boost::shared_ptr<RGBPatchFeatureCalculator> RGBPatchFeatureCalculator_Ptr;
typedef boost::shared_ptr<const RGBPatchFeatureCalculator> RGBPatchFeatureCalculator_CPtr;

typedef RGBDPatchFeatureCalculator<Keypoint3DColour,RGBDPatchDescriptor> DA_RGBDPatchFeatureCalculator;
typedef boost::shared_ptr<DA_RGBDPatchFeatureCalculator> DA_RGBDPatchFeatureCalculator_Ptr;
typedef boost::shared_ptr<const DA_RGBDPatchFeatureCalculator> DA_RGBDPatchFeatureCalculator_CPtr;

}

#endif
