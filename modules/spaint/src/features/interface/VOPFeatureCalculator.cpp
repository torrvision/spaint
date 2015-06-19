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

VOPFeatureCalculator::VOPFeatureCalculator(size_t maxVoxelLocationCount, size_t patchSize, float patchSpacing, size_t binCount)
:
  // Debugging variables
  m_debugDelayMs(30),
  m_debuggingOutputWindowName("DebuggingOutputWindow"),

  // Normal variables
  m_binCount(binCount),
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
  process_debug_window();
#endif

  // Calculate the surface normals at the voxel locations (this also writes them into the feature vectors).
  const SpaintVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
  const ITMVoxelIndex::IndexData *indexData = scene->index.getIndexData();
  calculate_surface_normals(voxelLocationsMB, voxelData, indexData, featuresMB);

  // Construct a coordinate system in the tangent plane to the surface at each voxel location.
  const int voxelLocationCount = static_cast<int>(voxelLocationsMB.dataSize);
  generate_coordinate_systems(voxelLocationCount);

  // Read an RGB patch around each voxel location.
  generate_rgb_patches(voxelLocationsMB, voxelData, indexData, featuresMB);

#if defined(WITH_OPENCV) && DEBUG_FEATURE_DISPLAY
  display_features(featuresMB, voxelLocationCount, "Feature Samples Before Rotation");
#endif

  // Determine the dominant orientation for each patch and update the coordinate systems accordingly.
  update_coordinate_systems(voxelLocationCount, featuresMB);

  // Read a new RGB patch around each voxel location that is oriented based on the dominant orientation.
  generate_rgb_patches(voxelLocationsMB, voxelData, indexData, featuresMB);

#if defined(WITH_OPENCV) && DEBUG_FEATURE_DISPLAY
  display_features(featuresMB, voxelLocationCount, "Feature Samples After Rotation");
#endif

  // Convert the new RGB patches to the CIELab colour space.
  convert_patches_to_lab(voxelLocationCount, featuresMB);

#if defined(WITH_OPENCV) && DEBUG_FEATURE_DISPLAY
  display_features(featuresMB, voxelLocationCount, "Feature Samples After LAB Conversion");
#endif

  // For each feature vector, fill in the height of the corresponding voxel in the scene as an extra feature.
  // Since we assume that the scene's up vector corresponds with the up vector in the real world, the heights
  // we write are simply the y values of the voxels. We ensure that scene up corresponds to world up by making
  // use of the gyro in the Oculus Rift. (If the Rift is not being used, the camera should simply be held
  // horizontally when running the application.)
  fill_in_heights(voxelLocationsMB, featuresMB);
}

size_t VOPFeatureCalculator::get_feature_count() const
{
  // A feature vector consists of a patch of CIELab colour values, the surface normal,
  // and the height of the voxel in the scene.
  return m_patchSize * m_patchSize * 3 + 3 + 1;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void VOPFeatureCalculator::display_features(const ORUtils::MemoryBlock<float>& featuresMB, int voxelLocationCount, const std::string& windowName) const
{
#if defined(WITH_OPENCV) && DEBUG_FEATURE_DISPLAY
  const float *features = featuresMB.GetData(MEMORYDEVICE_CPU);
  const int patchSize = static_cast<int>(m_patchSize);

  std::vector<cv::Mat3b> rgbPatchImages(voxelLocationCount);
  for(int i = 0; i < voxelLocationCount; ++i)
  {
    rgbPatchImages[i] = OpenCVUtil::make_rgb_image(features + i * get_feature_count(), patchSize, patchSize);
  }

  const size_t scaleFactor = 6;
  const size_t scaledPatchSize = scaleFactor * m_patchSize;
  cv::Mat3b tiledImage = OpenCVUtil::tile_image_patches_bounded(rgbPatchImages, 1024, 768, scaledPatchSize, scaledPatchSize);
  cv::imshow(windowName, tiledImage);
#endif
}

void VOPFeatureCalculator::process_debug_window() const
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
