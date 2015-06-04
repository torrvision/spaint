/**
 * spaint: VOPFeatureCalculator.cpp
 */

#include "features/interface/VOPFeatureCalculator.h"

#ifdef WITH_OPENCV
#include "ocv/OpenCVUtil.h"
#endif

namespace spaint {

//#################### CONSTRUCTORS ####################

VOPFeatureCalculator::VOPFeatureCalculator(size_t maxVoxelLocationCount, size_t patchSize, float patchSpacing)
: m_patchSize(patchSize),
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
  // Calculate the surface normals at the voxel locations.
  const SpaintVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
  const ITMVoxelIndex::IndexData *indexData = scene->index.getIndexData();
  calculate_surface_normals(voxelLocationsMB, voxelData, indexData);

  // Construct a coordinate system in the tangent plane to the surface at each voxel location.
  const int voxelLocationCount = static_cast<int>(voxelLocationsMB.dataSize);
  generate_coordinate_systems(voxelLocationCount);

  // Read an RGB patch around each voxel location.
  generate_rgb_patches(voxelLocationsMB, voxelData, indexData, featuresMB);

#if 1 && defined(WITH_OPENCV)
  debug_display_features(featuresMB.GetData(MEMORYDEVICE_CPU), voxelLocationsMB.dataSize, "Feature Samples Before Rotation");
#endif

  // Determine the dominant orientation for each patch and update the coordinate systems accordingly.
  update_coordinate_systems(voxelLocationCount, featuresMB);

  // Read a new RGB patch around each voxel location that is oriented based on the dominant orientation.
  generate_rgb_patches(voxelLocationsMB, voxelData, indexData, featuresMB);

#if 1 && defined(WITH_OPENCV)
  debug_display_features(featuresMB.GetData(MEMORYDEVICE_CPU), voxelLocationsMB.dataSize, "Feature Samples After Rotation");
#endif

  // Convert the new RGB patches to the CIELab colour space to form the feature vectors.
  convert_patches_to_lab(voxelLocationCount, featuresMB);

#if 1 && defined(WITH_OPENCV)
  debug_display_features(featuresMB.GetData(MEMORYDEVICE_CPU), voxelLocationsMB.dataSize, "Feature Samples After LAB Conversion");
#endif

  // For each feature vector, fill in the surface normal and the signed distance to the dominant horizontal surface present in the scene as extra features.
  // TODO
}

size_t VOPFeatureCalculator::get_feature_count() const
{
  // A feature vector consists of a patch of CIELab colour values, the surface normal,
  // and the signed distance to the dominant horizontal surface present in the scene.
  return m_patchSize * m_patchSize * 3 + 3 + 1;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

#ifdef WITH_OPENCV
void VOPFeatureCalculator::debug_display_features(const float *features, size_t size, const std::string& windowName) const
{
  std::vector<cv::Mat3b> rgbPatchImages(size);
  for(size_t i = 0; i < size; ++i)
  {
    rgbPatchImages[i] = OpenCVUtil::make_image_rgb_cpu(features + i * get_feature_count(), m_patchSize, m_patchSize);
  }
  static bool initialised = false;
  static int debugDelay = 30; //milliseconds
  if(!initialised)
  {
    cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("Debug delay (milliseconds)", windowName, &debugDelay, 3000);
  }
  debugDelay = cv::getTrackbarPos("Debug delay (milliseconds)", windowName);
  const size_t tilingWidth = 17;
  const size_t tilingHeight = 9;
  const size_t scaleFactor = 6;
  cv::Mat3b tiledImage = OpenCVUtil::tile_images(rgbPatchImages, tilingWidth, tilingHeight, scaleFactor);
  cv::imshow(windowName, tiledImage);
  cv::waitKey(debugDelay);
}
#endif
}
