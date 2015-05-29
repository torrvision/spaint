/**
 * spaint: VOPFeatureCalculator_CPU.cpp
 */

#include "features/cpu/VOPFeatureCalculator_CPU.h"

#include <cmath>
#include <vector>

#include <ITMLib/Engine/DeviceAgnostic/ITMRepresentationAccess.h>

#include "features/shared/VOPFeatureCalculator_Shared.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

VOPFeatureCalculator_CPU::VOPFeatureCalculator_CPU(size_t maxVoxelLocationCount, size_t patchSize, float patchSpacing)
: VOPFeatureCalculator(maxVoxelLocationCount, patchSize, patchSpacing)
{}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void VOPFeatureCalculator_CPU::calculate_surface_normals(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB,
                                                         const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData) const
{
  Vector3f *surfaceNormals = m_surfaceNormalsMB.GetData(MEMORYDEVICE_CPU);
  const Vector3s *voxelLocations = voxelLocationsMB.GetData(MEMORYDEVICE_CPU);
  const int voxelLocationCount = static_cast<int>(voxelLocationsMB.dataSize);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int voxelLocationIndex = 0; voxelLocationIndex < voxelLocationCount; ++voxelLocationIndex)
  {
    write_surface_normal(voxelLocationIndex, voxelLocations, voxelData, indexData, surfaceNormals);
  }
}

void VOPFeatureCalculator_CPU::convert_patches_to_lab(int voxelLocationCount, ORUtils::MemoryBlock<float>& featuresMB) const
{
  const size_t featureCount = get_feature_count();
  float *features = featuresMB.GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int voxelLocationIndex = 0; voxelLocationIndex < voxelLocationCount; ++voxelLocationIndex)
  {
    convert_patch_to_lab(voxelLocationIndex, featureCount, features);
  }
}

void VOPFeatureCalculator_CPU::generate_coordinate_systems(int voxelLocationCount) const
{
  const Vector3f *surfaceNormals = m_surfaceNormalsMB.GetData(MEMORYDEVICE_CPU);
  Vector3f *xAxes = m_xAxesMB.GetData(MEMORYDEVICE_CPU);
  Vector3f *yAxes = m_yAxesMB.GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int voxelLocationIndex = 0; voxelLocationIndex < voxelLocationCount; ++voxelLocationIndex)
  {
    generate_coordinate_system(voxelLocationIndex, surfaceNormals, xAxes, yAxes);
  }
}

void VOPFeatureCalculator_CPU::generate_rgb_patches(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB,
                                                    const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData,
                                                    ORUtils::MemoryBlock<float>& featuresMB) const
{
  const size_t featureCount = get_feature_count();
  float *features = featuresMB.GetData(MEMORYDEVICE_CPU);
  const Vector3f *xAxes = m_xAxesMB.GetData(MEMORYDEVICE_CPU);
  const Vector3f *yAxes = m_yAxesMB.GetData(MEMORYDEVICE_CPU);
  const Vector3s *voxelLocations = voxelLocationsMB.GetData(MEMORYDEVICE_CPU);
  const int voxelLocationCount = static_cast<int>(voxelLocationsMB.dataSize);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int voxelLocationIndex = 0; voxelLocationIndex < voxelLocationCount; ++voxelLocationIndex)
  {
    generate_rgb_patch(voxelLocationIndex, voxelLocations, xAxes, yAxes, voxelData, indexData, m_patchSize, m_patchSpacing, featureCount, features);
  }
}

void VOPFeatureCalculator_CPU::update_coordinate_systems(int voxelLocationCount, const ORUtils::MemoryBlock<float>& featuresMB) const
{
  const size_t binCount = 32;
  const float *features = featuresMB.GetData(MEMORYDEVICE_CPU);
  const size_t featureCount = get_feature_count();
  const size_t patchArea = m_patchSize * m_patchSize;
  Vector3f *xAxes = m_xAxesMB.GetData(MEMORYDEVICE_CPU);
  Vector3f *yAxes = m_yAxesMB.GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  //#pragma omp parallel for
#endif
  for(int voxelLocationIndex = 0; voxelLocationIndex < voxelLocationCount; ++voxelLocationIndex)
  {
    const float *featuresForVoxel = features + voxelLocationIndex * featureCount;
    std::vector<size_t> histogram(binCount);
    std::vector<float> intensities(patchArea);

    for(size_t i = 0; i < patchArea; ++i)
    {
      size_t y = i / m_patchSize;
      size_t x = i % m_patchSize;
      size_t offset = y * m_patchSize + x;
      size_t featureOffset = offset * 3;

      float r = featuresForVoxel[featureOffset];
      float g = featuresForVoxel[featureOffset + 1];
      float b = featuresForVoxel[featureOffset + 2];

      // TODO: Consider alternatives.
      intensities[offset] = (r + g + b) / 3.0f;
    }

    for(size_t i = 0; i < patchArea; ++i)
    {
      size_t y = i / m_patchSize;
      size_t x = i % m_patchSize;

      if(x != 0 && y != 0 && x != m_patchSize - 1 && y != m_patchSize - 1)
      {
        size_t offset = y * m_patchSize + x;

        // Compute the derivatives.
        float xDeriv = intensities[offset + 1] - intensities[offset - 1];
        float yDeriv = intensities[offset + m_patchSize] - intensities[offset - m_patchSize];

        // Compute the orientation.
        double ori = atan2(yDeriv, xDeriv) + 2 * M_PI;

        // Quantize the orientation and update the histogram.
        int bin = static_cast<int>(binCount * ori / (2 * M_PI)) % binCount;
        ++histogram[bin]; // note: this will need synchronisation on the GPU
      }
    }

    size_t dominantOrientationBin;
    size_t highestCount = 0;
    for(size_t i = 0; i < binCount; ++i)
    {
      size_t currentCount = histogram[i];
      if(currentCount > highestCount)
      {
        highestCount = currentCount;
        dominantOrientationBin = i;
      }
    }

    float binAngle = static_cast<float>(2 * M_PI) / binCount;
    float dominantOrientation = dominantOrientationBin * binAngle;

    float c = cos(dominantOrientation);
    float s = sin(dominantOrientation);

    Vector3f xAxis = xAxes[voxelLocationIndex];
    Vector3f yAxis = yAxes[voxelLocationIndex];

    xAxes[voxelLocationIndex] = c * xAxis + s * yAxis;
    yAxes[voxelLocationIndex] = c * yAxis - s * xAxis;
  }
}

}
