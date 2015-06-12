/**
 * spaint: VOPFeatureCalculator.h
 */

#ifndef H_SPAINT_VOPFEATURECALCULATOR
#define H_SPAINT_VOPFEATURECALCULATOR

#include "FeatureCalculator.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to calculate VOP feature descriptors for voxels sampled from a scene.
 */
class VOPFeatureCalculator : public FeatureCalculator
{
  //#################### PRIVATE DEBUGGING VARIABLES ####################
private:
  /** The number of milliseconds by which to delay between consecutive frames when debugging (0 = pause). */
  mutable int m_debugDelayMs;

  /** The name of the debugging output window. */
  std::string m_debuggingOutputWindowName;

  //#################### PROTECTED VARIABLES ####################
protected:
  /** The side length of a VOP patch (must be odd). */
  size_t m_patchSize;

  /** The spacing in the scene between individual pixels in a patch. */
  float m_patchSpacing;

  /** The surface normals at the voxel locations. */
  mutable ORUtils::MemoryBlock<Vector3f> m_surfaceNormalsMB;

  /** The x axes of the coordinate systems in the tangent planes to the surfaces at the voxel locations. */
  mutable ORUtils::MemoryBlock<Vector3f> m_xAxesMB;

  /** The y axes of the coordinate systems in the tangent planes to the surfaces at the voxel locations. */
  mutable ORUtils::MemoryBlock<Vector3f> m_yAxesMB;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a VOP feature calculator.
   *
   * \param maxVoxelLocationCount The maximum number of voxel locations for which we will be calculating features at any one time.
   * \param patchSize             The side length of a VOP patch (must be odd).
   * \param patchSpacing          The spacing in the scene (in voxels) between individual pixels in a patch.
   */
  VOPFeatureCalculator(size_t maxVoxelLocationCount, size_t patchSize, float patchSpacing);

  //#################### PRIVATE ABSTRACT MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Calculates the surface normals at the voxel locations.
   *
   * \param voxelLocationsMB  A memory block containing the locations of the voxels for which to calculate the surface normals.
   * \param voxelData         The scene's voxel data.
   * \param indexData         The scene's index data.
   * \param featuresMB        A memory block into which to store the calculated feature descriptors (packed sequentially).
   */
  virtual void calculate_surface_normals(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData,
                                         ORUtils::MemoryBlock<float>& featuresMB) const = 0;

  /**
   * \brief TODO
   */
  virtual void convert_patches_to_lab(int voxelLocationCount, ORUtils::MemoryBlock<float>& featuresMB) const = 0;

  /**
   * \brief Generates coordinate systems in the tangent planes to the surfaces at the voxel locations.
   *
   * \param voxelLocationCount  The number of voxel locations for which we are calculating features.
   */
  virtual void generate_coordinate_systems(int voxelLocationCount) const = 0;

  /**
   * \brief TODO
   */
  virtual void generate_rgb_patches(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB,
                                    const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData,
                                    ORUtils::MemoryBlock<float>& featuresMB) const = 0;

  /**
   * \brief TODO
   */
  virtual void update_coordinate_systems(int voxelLocationCount, const ORUtils::MemoryBlock<float>& featuresMB) const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void calculate_features(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB,
                                  const ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene,
                                  ORUtils::MemoryBlock<float>& featuresMB) const;

  /** Override */
  virtual size_t get_feature_count() const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief TODO.
   */
  void debug_display_features(const ORUtils::MemoryBlock<float>& featuresMB, size_t size, const std::string& windowName) const;

  /**
   * \brief Sets up a debugging window containing a trackbar that can be used to control the delay between consecutive frames.
   */
  void process_debug_windows() const;
};

}

#endif
