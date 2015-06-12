/**
 * spaint: VOPFeatureCalculator.cpp
 */

#include "features/interface/VOPFeatureCalculator.h"

#ifdef WITH_OPENCV
#include "ocv/OpenCVUtil.h"
#endif

#define DEBUG_FEATURE_DISPLAY 0

namespace spaint {

//#################### CONSTRUCTORS ####################

VOPFeatureCalculator::VOPFeatureCalculator(size_t maxVoxelLocationCount, size_t patchSize, float patchSpacing)
:
  // Debugging Variables.
  m_debugDelayMs(30),
  m_debuggingOutputWindowName("DebuggingOutputWindow"),

  // Normal Variables.
  m_patchSize(patchSize),
  m_patchSpacing(patchSpacing),
  m_surfaceNormalsMB(maxVoxelLocationCount, true, true),
  m_xAxesMB(maxVoxelLocationCount, true, true),
  m_yAxesMB(maxVoxelLocationCount, true, true)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void VOPFeatureCalculator::calculate_features(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB,
                                              const ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene,
                                              ORUtils::MemoryBlock<float>& featuresMB) const
{
#if defined(WITH_OPENCV) && DEBUG_FEATURE_DISPLAY
  process_debug_windows();
#endif

  // Calculate the surface normals at the voxel locations.
  const SpaintVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
  const ITMVoxelIndex::IndexData *indexData = scene->index.getIndexData();
  calculate_surface_normals(voxelLocationsMB, voxelData, indexData, featuresMB);

  // Construct a coordinate system in the tangent plane to the surface at each voxel location.
  const int voxelLocationCount = static_cast<int>(voxelLocationsMB.dataSize);
  generate_coordinate_systems(voxelLocationCount);

  // Read an RGB patch around each voxel location.
  generate_rgb_patches(voxelLocationsMB, voxelData, indexData, featuresMB);

#if defined(WITH_OPENCV) && DEBUG_FEATURE_DISPLAY
  debug_display_features(featuresMB, voxelLocationsMB.dataSize, "Feature Samples Before Rotation");
#endif

  // Determine the dominant orientation for each patch and update the coordinate systems accordingly.
  update_coordinate_systems(voxelLocationCount, featuresMB);

  // Read a new RGB patch around each voxel location that is oriented based on the dominant orientation.
  generate_rgb_patches(voxelLocationsMB, voxelData, indexData, featuresMB);

#if defined(WITH_OPENCV) && DEBUG_FEATURE_DISPLAY
  debug_display_features(featuresMB, voxelLocationsMB.dataSize, "Feature Samples After Rotation");
#endif

  // Convert the new RGB patches to the CIELab colour space to form the feature vectors.
  convert_patches_to_lab(voxelLocationCount, featuresMB);

#if defined(WITH_OPENCV) && DEBUG_FEATURE_DISPLAY
  debug_display_features(featuresMB, voxelLocationsMB.dataSize, "Feature Samples After LAB Conversion");
#endif

  // For each feature vector, fill in the surface normal and the signed distance to the dominant horizontal surface present in the scene as extra features.
  //TODO fill_in_signed_distance().
}

size_t VOPFeatureCalculator::get_feature_count() const
{
  // A feature vector consists of a patch of CIELab colour values, the surface normal,
  // and the signed distance to the dominant horizontal surface present in the scene.
  return m_patchSize * m_patchSize * 3 + 3 + 1;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void VOPFeatureCalculator::debug_display_features(const ORUtils::MemoryBlock<float>& featuresMB, size_t size, const std::string& windowName) const
{
#if defined(WITH_OPENCV) && DEBUG_FEATURE_DISPLAY
  const float *features = featuresMB.GetData(MEMORYDEVICE_CPU);

  std::vector<cv::Mat3b> rgbPatchImages(size);
  for(size_t i = 0; i < size; ++i)
  {
    rgbPatchImages[i] = OpenCVUtil::make_image_rgb_cpu(features + i * get_feature_count(), m_patchSize, m_patchSize);
  }

  const size_t scaleFactor = 6;
  const size_t patchWidth = scaleFactor * m_patchSize;
  cv::Mat3b tiledImage = OpenCVUtil::tile_regular_image_patches_within_image_bounds(rgbPatchImages, 1024, 768, patchWidth, patchWidth);
  cv::imshow(windowName, tiledImage);
#endif
}

void VOPFeatureCalculator::process_debug_windows() const
{
#if defined(WITH_OPENCV) && DEBUG_FEATURE_DISPLAY
  // If this is the first iteration, create a debugging window with a trackbar used to control the delay between consecutive frames.
  static bool initialised = false;
  if(!initialised)
  {
    cv::namedWindow(m_debuggingOutputWindowName, cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("debugDelayMs", m_debuggingOutputWindowName, &m_debugDelayMs, 3000);
  }

  // Update the delay based on the value of the trackbar.
  m_debugDelayMs = cv::getTrackbarPos("debugDelayMs", m_debuggingOutputWindowName);

  // Wait for the specified number of milliseconds (or until a key is pressed).
  cv::waitKey(m_debugDelayMs);
#endif
}

}
